// src/app/task_sd.rs
//
// SD 卡只读支持：ESP32-S3 原生 SDMMC 1-bit（GOOUUU: CMD=IO38 CLK=IO39 DATA0=IO40）
// + sdio crate 做协议层枚举（CMD0/CMD8/ACMD41/CMD2/CMD3/CMD9/CMD7/ACMD51/ACMD6/
//   信号模式切换/CMD13/ACMD13/CMD16）+ 自实现 FAT16/FAT32 只读（列目录 / 读文件）。
//
// 【为什么换成 sdio + Async】
// 板级验证工程 esp32s3cam-sd 证明 SD 硬件本身没问题，但 esp-hal 只有 `Async` Slot
// 实现了 `sdio::MmcBus`；原先在 `Blocking` Slot 上手写 CMD 序列的做法缺少 CSD/SCR/
// 总线宽度/信号模式等枚举步骤，上机在枚举阶段就失败。
//
// 【并发模型】
// SD 命令是异步的（每条都等中断），不能像以前那样在同步上下文里直接摸 `static mut`。
// 因此由 `task_sd` 这个 embassy 任务独占 SDHOST 控制器与块设备：
//   - 其它异步任务（web）投 `SD_REQ` → 等 `SD_RESP`，先抢 `SD_CALL` 保证一问一答不串号；
//   - 卡状态与根目录列表缓存在 `SD_INFO` / `SD_FILES`，供同步上下文（shell、
//     /api/status）直接读，不会阻塞在执行器里。
use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::cell::RefCell;
use defmt::{error, warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex as AsyncMutex;

use aligned::{A4, Aligned};
use block_device_driver::BlockDevice as _;
use esp_hal::peripherals::{GPIO38, GPIO39, GPIO40, SDHOST, TIMG1};
use esp_hal::sdmmc::{Config, SdHostController, SlotConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::timer::OneShotTimer;
use sdio::sd::Card;
use sdio::Addressable as _;

/// 运行总线频率。1-bit 且只靠内部上拉时 20MHz 偏激进，读块出错可降到 10MHz
const SD_FREQ: u32 = 20_000_000;
/// 单次下载上限，避免把整张卡的巨型文件读进堆
pub const MAX_DOWNLOAD_BYTES: usize = 256 * 1024;
/// 簇链遍历步数上限，防止 FAT 环路把任务转死（参见摄像头任务的让出经验）
const MAX_CHAIN_STEPS: u32 = 100_000;

// ------------------------------------------------------------
// 对外数据结构 / 通信原语
// ------------------------------------------------------------

#[derive(Clone, Copy)]
pub struct SdInfo {
    pub present: bool,
    pub mounted: bool,
    pub fat_type: u8, // 16 / 32，0=未知
    pub total_mb: u32,
    pub free_mb: u32,
}

/// 投递给 `task_sd` 的请求
enum SdCmd {
    /// 重新枚举 + 重扫根目录（刷新 `SD_INFO` / `SD_FILES`），完成后回 `Ack`
    Rescan,
    /// 按名读文件，结果经 `SD_RESP` 回送
    Download(String),
}

/// `task_sd` 的应答
enum SdResp {
    Ack,
    File(Option<Vec<u8>>),
}

/// 请求队列（任务间投递）
static SD_REQ: Channel<CriticalSectionRawMutex, SdCmd, 4> = Channel::new();
/// 应答队列（容量 1，配合 `SD_CALL` 串行化即一问一答）
static SD_RESP: Channel<CriticalSectionRawMutex, SdResp, 1> = Channel::new();
/// 串行化所有 SD 请求，保证 `SD_RESP` 不会串号
static SD_CALL: AsyncMutex<CriticalSectionRawMutex, ()> = AsyncMutex::new(());

/// 卡状态快照（供 /api/status 与 shell 同步读）
static SD_INFO: Mutex<CriticalSectionRawMutex, RefCell<SdInfo>> =
    Mutex::new(RefCell::new(SdInfo {
        present: false,
        mounted: false,
        fat_type: 0,
        total_mb: 0,
        free_mb: 0,
    }));
/// 根目录列表快照（供 shell 同步读）
static SD_FILES: Mutex<CriticalSectionRawMutex, RefCell<Vec<(String, u32)>>> =
    Mutex::new(RefCell::new(Vec::new()));

// ------------------------------------------------------------
// 类型别名（与板级验证工程一致）
// ------------------------------------------------------------

/// SDHOST 异步槽位（esp-hal 的 `Slot` 实现了 sdio 的 `MmcBus`）
type SdSlot<'d> = esp_hal::sdmmc::Slot<'d, 0, esp_hal::Async>;
/// 延时源（TIMG1 一次性定时器，sdio 枚举过程需要）
type SdDelay<'d> = OneShotTimer<'d, esp_hal::Async>;
/// sdio 提供的 512 字节块设备
type SdDev<'d> = sdio::BlockDevice<Card, SdSlot<'d>, SdDelay<'d>, 512>;

/// 一个块（512 字节）。ESP32-S3 上 SDHOST 的 DMA 不走缓存，4 字节对齐即可
type OneBlock = Aligned<A4, [u8; 512]>;

/// FAT 引导参数（扇区号均相对分区起始）
#[allow(dead_code)]
struct Fat {
    fat32: bool,
    sectors_per_cluster: u32,
    fat_start_lba: u32,
    fat_size_sectors: u32,
    num_fats: u32,
    root_dir_lba: u32, // FAT16 固定根目录区起始
    root_dir_entries: u32,
    root_cluster: u32, // FAT32 根目录首簇
    data_start_lba: u32, // 簇 2 的起始 LBA
    total_sectors: u32,
}

/// 已挂载的文件系统：分区偏移 + FAT 参数
struct SdFs {
    part_lba: u32,
    fat: Fat,
}

// ------------------------------------------------------------
// worker 任务：独占 SDHOST 与块设备
// ------------------------------------------------------------

#[embassy_executor::task]
pub async fn task_sd(
    sdhost: SDHOST<'static>,
    clk: GPIO39<'static>,
    cmd: GPIO38<'static>,
    data0: GPIO40<'static>,
    timg1: TIMG1<'static>,
) {
    let req_rx = SD_REQ.receiver();
    let resp_tx = SD_RESP.sender();

    // slot 借用 controller、controller 持有外设，两者都必须活在本任务作用域内
    let controller = match SdHostController::new(sdhost, Config::default()) {
        Ok(c) => c,
        Err(e) => {
            error!("[sd] SDHOST init failed: {:?}", e);
            set_unmounted();
            serve_unavailable(req_rx, resp_tx).await;
        }
    };
    let timg = TimerGroup::new(timg1);
    let delay = OneShotTimer::new(timg.timer0).into_async();

    let slot = match controller.slot::<0>(SlotConfig::default()) {
        Ok(s) => s.with_clk(clk).with_cmd(cmd).with_data0(data0).into_async(),
        Err(e) => {
            error!("[sd] slot0 unavailable: {:?}", e);
            set_unmounted();
            serve_unavailable(req_rx, resp_tx).await;
        }
    };
    defmt::info!("[sd] SDHOST up: slot0, 1-bit, CLK=GPIO39 CMD=GPIO38 D0=GPIO40");

    // sdio 内部先在 400kHz 完成枚举，再切到 SD_FREQ
    // 注意：new() 会吃掉 slot，失败后无法重试，开机无卡需重启才能用上
    let mut dev = match sdio::BlockDevice::<Card, _, _, 512>::new(slot, delay, SD_FREQ).await {
        Ok(d) => d,
        Err(e) => {
            warn!("[sd] card init failed (card absent?): {:?}", e);
            set_unmounted();
            serve_unavailable(req_rx, resp_tx).await;
        }
    };
    defmt::info!(
        "[sd] card ready, bus={} Hz, blocks={}",
        dev.freq(),
        dev.card().block_count()
    );

    // 首次挂载 + 扫目录，结果放进共享缓存
    let mut fs = mount(&mut dev).await;

    loop {
        // 只有本任务持有 dev，故 await 期间不会与其它 SD 操作交错
        match req_rx.receive().await {
            SdCmd::Rescan => {
                match dev.reacquire(SD_FREQ).await {
                    Ok(()) => fs = mount(&mut dev).await,
                    Err(e) => {
                        warn!("[sd] reacquire failed (card removed?): {:?}", e);
                        fs = None;
                        set_unmounted();
                    }
                }
                resp_tx.send(SdResp::Ack).await;
            }
            SdCmd::Download(name) => {
                let data = match &fs {
                    Some(f) => download(&mut dev, f, &name).await,
                    None => None,
                };
                resp_tx.send(SdResp::File(data)).await;
            }
        }
    }
}

/// SDHOST 或卡不可用（初始化失败）时也要应答所有请求，
/// 否则调用方会永久等在 `SD_RESP` 上并拖死 `SD_CALL` 互斥锁
async fn serve_unavailable(
    rx: Receiver<'static, CriticalSectionRawMutex, SdCmd, 4>,
    tx: Sender<'static, CriticalSectionRawMutex, SdResp, 1>,
) -> ! {
    loop {
        match rx.receive().await {
            SdCmd::Rescan => tx.send(SdResp::Ack).await,
            SdCmd::Download(_) => tx.send(SdResp::File(None)).await,
        }
    }
}

/// 读引导扇区 / BPB 挂载 FAT，并把状态与根目录列表写入共享缓存
async fn mount(dev: &mut SdDev<'_>) -> Option<SdFs> {
    let b0 = read_block(dev, 0).await?;
    let (part_lba, mbr_sectors) = parse_mbr(&b0);
    let bs = read_block(dev, part_lba).await?;
    let fat = match parse_bpb(&bs, mbr_sectors) {
        Ok(f) => f,
        Err(e) => {
            warn!("[sd] {}", e);
            set_unmounted();
            return None;
        }
    };
    let fs = SdFs { part_lba, fat };

    let total_mb = (fs.fat.total_sectors as u64 * 512 / 1024 / 1024) as u32;
    defmt::info!(
        "[sd] mounted FAT{} part_lba={} total={} MB",
        if fs.fat.fat32 { 32 } else { 16 },
        fs.part_lba,
        total_mb
    );
    SD_INFO.lock(|i| {
        *i.borrow_mut() = SdInfo {
            present: true,
            mounted: true,
            fat_type: if fs.fat.fat32 { 32 } else { 16 },
            total_mb,
            free_mb: 0,
        }
    });

    let hits = scan_root(dev, &fs).await;
    defmt::info!("[sd] root has {} file(s)", hits.len());
    SD_FILES.lock(|f| {
        *f.borrow_mut() = hits.into_iter().map(|h| (h.name, h.size)).collect()
    });
    Some(fs)
}

fn set_unmounted() {
    SD_INFO.lock(|i| {
        *i.borrow_mut() = SdInfo {
            present: false,
            mounted: false,
            fat_type: 0,
            total_mb: 0,
            free_mb: 0,
        }
    });
    SD_FILES.lock(|f| f.borrow_mut().clear());
}

// ------------------------------------------------------------
// 块访问原语
// ------------------------------------------------------------

/// 读单个 512 字节块；失败返回 None
async fn read_block(dev: &mut SdDev<'_>, lba: u32) -> Option<[u8; 512]> {
    let mut buf: Box<[OneBlock; 1]> = Box::new([Aligned([0u8; 512])]);
    dev.read(lba, buf.as_mut()).await.ok()?;
    let mut out = [0u8; 512];
    out.copy_from_slice(&buf[0][..]);
    Some(out)
}

// ------------------------------------------------------------
// MBR / BPB 解析
// ------------------------------------------------------------

/// 返回 (FAT 引导扇区所在 LBA, 分区扇区数)。超级软盘格式下引导扇区就是块 0。
fn parse_mbr(b0: &[u8; 512]) -> (u32, u32) {
    // 块 0 以 0xEB 跳转开头说明它本身就是引导扇区（无分区表）
    if !(b0[510] == 0x55 && b0[511] == 0xAA && b0[450] != 0xEB) {
        return (0, 0);
    }
    let p = &b0[446..462];
    let start = u32::from_le_bytes([p[8], p[9], p[10], p[11]]);
    let count = u32::from_le_bytes([p[12], p[13], p[14], p[15]]);
    if start == 0 || count == 0 {
        return (0, 0);
    }
    (start, count)
}

/// 解析 BPB（FAT12 不支持，会被判为非法）
fn parse_bpb(bs: &[u8; 512], mbr_sectors: u32) -> Result<Fat, &'static str> {
    if bs[510] != 0x55 || bs[511] != 0xAA {
        return Err("no 0x55AA signature");
    }
    if u16::from_le_bytes([bs[11], bs[12]]) as u32 != 512 {
        return Err("non-512 sector");
    }
    let spc = bs[13] as u32;
    let reserved = u16::from_le_bytes([bs[14], bs[15]]) as u32;
    let num_fats = (bs[16] as u32).max(1);
    let root_entries = u16::from_le_bytes([bs[17], bs[18]]) as u32;
    let total16 = u16::from_le_bytes([bs[19], bs[20]]) as u32;
    let spf16 = u16::from_le_bytes([bs[21], bs[22]]) as u32;
    let total32 = u32::from_le_bytes([bs[32], bs[33], bs[34], bs[35]]);
    let spf32 = u32::from_le_bytes([bs[36], bs[37], bs[38], bs[39]]);
    let root_cluster = u32::from_le_bytes([bs[44], bs[45], bs[46], bs[47]]);

    // 文件系统标记：FAT12/16 在 0x36，FAT32 在 0x52
    if &bs[0x36..0x39] != b"FAT" && &bs[0x52..0x55] != b"FAT" {
        return Err("no FAT signature");
    }
    if spc == 0 {
        return Err("bad sectors-per-cluster");
    }

    let total = if total32 != 0 {
        total32
    } else if total16 != 0 {
        total16
    } else {
        mbr_sectors
    };

    let fat32 = root_entries == 0 && spf16 == 0;
    let fat_size = if fat32 { spf32 } else { spf16 };
    if fat_size == 0 {
        return Err("zero FAT size");
    }
    // FAT16 的根目录区夹在 FAT 与数据区之间；FAT32 的数据区直接跟在 FAT 后面
    let root_dir_sectors = if fat32 {
        0
    } else {
        (root_entries * 32).div_ceil(512)
    };
    let data_start = reserved + num_fats * fat_size + root_dir_sectors;

    Ok(Fat {
        fat32,
        sectors_per_cluster: spc,
        fat_start_lba: reserved,
        fat_size_sectors: fat_size,
        num_fats,
        root_dir_lba: reserved + num_fats * fat_size,
        root_dir_entries: root_entries,
        root_cluster: if fat32 { root_cluster.max(2) } else { 2 },
        data_start_lba: data_start,
        total_sectors: total,
    })
}

fn cluster_to_lba(fs: &SdFs, cluster: u32) -> u32 {
    fs.part_lba + fs.fat.data_start_lba + (cluster - 2) * fs.fat.sectors_per_cluster
}

fn is_eoc(fs: &SdFs, cluster: u32) -> bool {
    if fs.fat.fat32 {
        cluster >= 0x0FFF_FFF8
    } else {
        cluster >= 0xFFF8
    }
}

/// FAT16 固定根目录区占多少扇区
fn root_dir_sectors(fat: &Fat) -> u32 {
    (fat.root_dir_entries * 32).div_ceil(512)
}

/// 读 FAT 表中 `cluster` 的下一簇号（取 FAT0），出错按 EOC 处理
async fn next_cluster(dev: &mut SdDev<'_>, fs: &SdFs, cluster: u32) -> u32 {
    let fat32 = fs.fat.fat32;
    let byte_off = if fat32 { cluster * 4 } else { cluster * 2 };
    // 512 能被 2 / 4 整除，表项不会跨扇区
    let lba = fs.part_lba + fs.fat.fat_start_lba + byte_off / 512;
    match read_block(dev, lba).await {
        Some(b) => {
            let idx = (byte_off % 512) as usize;
            if fat32 {
                u32::from_le_bytes([b[idx], b[idx + 1], b[idx + 2], b[idx + 3]]) & 0x0FFF_FFFF
            } else {
                u16::from_le_bytes([b[idx], b[idx + 1]]) as u32
            }
        }
        None => 0x0FFF_FFFF,
    }
}

// ------------------------------------------------------------
// 目录 / 文件
// ------------------------------------------------------------

/// 一个普通文件目录项的解析结果
struct DirHit {
    name: String,
    size: u32,
    cluster: u32,
}

/// 长文件名片段收集器：(序号, 13 个 UTF-16LE 码元)
struct Lfn {
    parts: Vec<(u8, [u16; 13])>,
}

impl Lfn {
    const fn new() -> Self {
        Self { parts: Vec::new() }
    }

    fn push(&mut self, e: &[u8]) {
        self.parts.push((e[0] & 0x3F, lfn_units(e)));
    }

    /// 按序号拼接出长名并清空收集器；无片段返回 None
    fn take(&mut self) -> Option<String> {
        if self.parts.is_empty() {
            return None;
        }
        self.parts.sort_by_key(|(seq, _)| *seq);
        let mut units: Vec<u16> = Vec::with_capacity(self.parts.len() * 13);
        for (_, u) in self.parts.iter() {
            units.extend_from_slice(u);
        }
        self.parts.clear();
        Some(decode_lfn(&units))
    }
}

/// 扫描一个目录扇区（16 个 32 字节条目）
fn scan_block(b: &[u8; 512], lfn: &mut Lfn, fat32: bool, out: &mut Vec<DirHit>) {
    for e in b.chunks_exact(32) {
        if e[0] == 0x00 {
            break; // 空槽，后面无条目
        }
        if e[0] == 0xE5 {
            continue; // 已删除
        }
        let attr = e[11];
        if attr == 0x0F {
            lfn.push(e); // 长文件名片段
            continue;
        }
        let name = lfn.take().unwrap_or_else(|| short_name(e));
        if attr & 0x08 != 0 || attr & 0x10 != 0 {
            continue; // VOLUME_ID / 子目录
        }
        let lo = u16::from_le_bytes([e[26], e[27]]) as u32;
        let hi = u16::from_le_bytes([e[20], e[21]]) as u32;
        out.push(DirHit {
            name,
            size: u32::from_le_bytes([e[28], e[29], e[30], e[31]]),
            cluster: if fat32 { (hi << 16) | lo } else { lo },
        });
    }
}

/// 遍历根目录，收集所有普通文件
async fn scan_root(dev: &mut SdDev<'_>, fs: &SdFs) -> Vec<DirHit> {
    let mut out = Vec::new();
    let mut lfn = Lfn::new();

    if fs.fat.fat32 {
        let mut cluster = fs.fat.root_cluster;
        let mut steps = 0u32;
        while cluster >= 2 && !is_eoc(fs, cluster) {
            let lba = cluster_to_lba(fs, cluster);
            for s in 0..fs.fat.sectors_per_cluster {
                if let Some(b) = read_block(dev, lba + s).await {
                    scan_block(&b, &mut lfn, true, &mut out);
                }
            }
            cluster = next_cluster(dev, fs, cluster).await;
            steps += 1;
            if steps > MAX_CHAIN_STEPS {
                warn!("[sd] root cluster chain too long, abort scan");
                break;
            }
        }
    } else {
        let start = fs.part_lba + fs.fat.root_dir_lba;
        for s in 0..root_dir_sectors(&fs.fat) {
            if let Some(b) = read_block(dev, start + s).await {
                scan_block(&b, &mut lfn, false, &mut out);
            }
        }
    }
    out
}

/// 按簇链读出文件内容（截断到 `cap`）
async fn read_clusters(
    dev: &mut SdDev<'_>,
    fs: &SdFs,
    start_cluster: u32,
    size: u32,
    cap: usize,
) -> Option<Vec<u8>> {
    let want = (size as usize).min(cap);
    let mut data: Vec<u8> = Vec::new();
    let mut cluster = start_cluster;
    let mut steps = 0u32;
    while data.len() < want && cluster >= 2 && !is_eoc(fs, cluster) {
        let lba = cluster_to_lba(fs, cluster);
        for s in 0..fs.fat.sectors_per_cluster {
            if data.len() >= want {
                break;
            }
            let b = read_block(dev, lba + s).await?;
            let n = core::cmp::min(512, want - data.len());
            data.extend_from_slice(&b[..n]);
        }
        cluster = next_cluster(dev, fs, cluster).await;
        steps += 1;
        if steps > MAX_CHAIN_STEPS {
            warn!("[sd] file cluster chain too long");
            break;
        }
    }
    data.truncate(want);
    Some(data)
}

/// 按短名或长名（大小写不敏感）读一个根目录文件
async fn download(dev: &mut SdDev<'_>, fs: &SdFs, want: &str) -> Option<Vec<u8>> {
    let hit = scan_root(dev, fs)
        .await
        .into_iter()
        .find(|h| h.name.eq_ignore_ascii_case(want))?;
    read_clusters(dev, fs, hit.cluster, hit.size, MAX_DOWNLOAD_BYTES).await
}

/// 8.3 短名转小写字符串（去尾部补空格）
fn short_name(e: &[u8]) -> String {
    let mut s = String::new();
    for &c in &e[0..8] {
        if c == b' ' {
            break;
        }
        s.push((c as char).to_ascii_lowercase());
    }
    let ext = &e[8..11];
    if ext[0] != b' ' {
        s.push('.');
        for &c in ext {
            if c == b' ' {
                break;
            }
            s.push((c as char).to_ascii_lowercase());
        }
    }
    s
}

/// 从一个 LFN 条目取回 13 个 UTF-16LE 码元（字节偏移见 SD 物理层规范）
fn lfn_units(e: &[u8]) -> [u16; 13] {
    const OFF: [usize; 13] = [1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30];
    let mut out = [0u16; 13];
    for (i, &o) in OFF.iter().enumerate() {
        out[i] = u16::from_le_bytes([e[o], e[o + 1]]);
    }
    out
}

/// 把 UTF-16LE 码元解码成字符串（可打印 ASCII 保留，其它记为 `?`）
fn decode_lfn(units: &[u16]) -> String {
    let mut s = String::new();
    for &u in units {
        if u == 0 {
            break;
        }
        if u == 0xFFFF {
            continue;
        }
        s.push(if (0x20..=0x7E).contains(&u) { u as u8 as char } else { '?' });
    }
    s
}

// ------------------------------------------------------------
// 对外 API
// ------------------------------------------------------------

/// 缓存的卡状态（同步，供 /api/status 与 shell）
pub fn info() -> SdInfo {
    SD_INFO.lock(|i| *i.borrow())
}

/// 缓存的根目录列表（同步，供 shell，不触发读卡）
pub fn cached_files() -> Vec<(String, u32)> {
    SD_FILES.lock(|f| f.borrow().clone())
}

/// 非阻塞地请求后台重扫（同步上下文如 shell 用；结果下次读取时生效）
pub fn request_rescan() {
    let _ = SD_REQ.try_send(SdCmd::Rescan);
}

/// 重新枚举并重扫根目录，返回最新列表（异步，供 web）
pub async fn list_files() -> Vec<(String, u32)> {
    let _guard = SD_CALL.lock().await;
    SD_REQ.sender().send(SdCmd::Rescan).await;
    let _ = SD_RESP.receiver().receive().await;
    cached_files()
}

/// 读取根目录下的文件内容（异步，供 web）
pub async fn read_file(name: &str) -> Option<Vec<u8>> {
    let _guard = SD_CALL.lock().await;
    SD_REQ.sender().send(SdCmd::Download(name.to_string())).await;
    match SD_RESP.receiver().receive().await {
        SdResp::File(data) => data,
        SdResp::Ack => None,
    }
}
