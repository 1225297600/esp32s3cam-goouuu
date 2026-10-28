// src/app/task_sd.rs
//
// SD 卡只读支持：ESP32-S3 原生 SDMMC 1-bit（GOOUUU: CMD=IO38 CLK=IO39 DATA0=IO40，
// 经 GPIO 矩阵）+ 自实现 FAT16/FAT32 只读（列目录 / 读文件）。
//
// 说明：SDMMC 的 command_blocking / read_blocks_blocking 为同步阻塞调用；
// esp-rtos 线程模式为单线程协作式执行器，SD 操作内部无 await，
// 因此一次 list/read 期间不会与其它任务交错，可安全使用 static mut 全局。
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use defmt::{info, warn};
use esp_hal::dma::aligned::DmaAlignedMut;
use esp_hal::peripherals::SDHOST;
use esp_hal::sdmmc::{
    BusWidth, CommandFlags, Config, ResponseLen, SdHostController, Slot, SlotConfig,
};
use static_cell::StaticCell;

/// 64 字节对齐的块缓冲（S3 内部 RAM 有缓存，DMA 需 64 对齐）
#[repr(C, align(64))]
struct AlignedBlock(pub [u8; 512]);

pub struct SdInfo {
    pub present: bool,
    pub mounted: bool,
    pub fat_type: u8, // 16 / 32，0=未知
    pub total_mb: u32,
    pub free_mb: u32,
}

struct Fat {
    fat32: bool,
    sectors_per_cluster: u32,
    fat_start_lba: u32,
    fat_size_sectors: u32,
    num_fats: u32,
    root_dir_lba: u32,    // FAT16 固定根目录起始；FAT32 用 root_cluster
    root_cluster: u32,    // FAT32 根目录首簇
    data_start_lba: u32,  // 簇 2 的起始 LBA
    total_sectors: u32,
    cluster_count: u32,
}

pub struct SdCard {
    slot: Slot<'static, 0, esp_hal::Blocking>,
    part_lba: u32,
    fat: Fat,
}

static mut CARD: Option<SdCard> = None;
static SDHOST_CTRL: StaticCell<SdHostController<'static>> = StaticCell::new();

const FLAGS: CommandFlags = CommandFlags {
    wait_complete: true,
    stop_abort: false,
    busy: false,
};

// ------------------------------------------------------------
// 初始化
// ------------------------------------------------------------

impl SdCard {
    /// 上电初始化 SD 卡并挂载 FAT。失败返回 Err（无卡 / 非 FAT）。
    pub fn init(
        sdhost: SDHOST<'static>,
        clk: esp_hal::peripherals::GPIO39<'static>,
        cmd: esp_hal::peripherals::GPIO38<'static>,
        data0: esp_hal::peripherals::GPIO40<'static>,
    ) -> Result<Self, &'static str> {
        let ctrl = SDHOST_CTRL.init(SdHostController::new(sdhost, Config::default()).map_err(|_| "sdhost init")?);
        let mut slot = ctrl
            .slot::<0>(SlotConfig::default())
            .map_err(|_| "slot")?
            .with_clk(clk)
            .with_cmd(cmd)
            .with_data0(data0);

        // 初始化时钟 ≤ 400kHz
        slot.set_bus_low_level(BusWidth::Bit1, 400_000).map_err(|_| "bus")?;
        nb_delay_ms(50); // 上电稳定（SD 卡 VDD 建立需时间）
        slot.send_init_sequence().map_err(|_| "init seq")?;
        nb_delay_ms(5);

        // CMD0 GO_IDLE_STATE（重试若干次，容忍上电初期无响应）
        let mut cmd0_ok = false;
        for i in 0..5 {
            match slot.command_blocking(0, 0, ResponseLen::Short, false, FLAGS) {
                Ok(_) => {
                    cmd0_ok = true;
                    break;
                }
                Err(e) => {
                    info!("[sd] CMD0 try {} err: {:?}", i, e);
                    nb_delay_ms(10);
                }
            }
        }
        if !cmd0_ok {
            return Err("CMD0");
        }
        // CMD8 SEND_IF_COND（判断 v2）
        let v2 = slot
            .command_blocking(8, 0x1AA, ResponseLen::Short, false, FLAGS)
            .map(|r| (r[0] & 0xFFF) == 0x1AA)
            .unwrap_or(false);

        // ACMD41 轮询就绪（HCS=bit30）
        let ocr_hcs = if v2 { 0x4000_0000 } else { 0 };
        let mut ready = false;
        for _ in 0..100 {
            slot.command_blocking(55, 0, ResponseLen::Short, false, FLAGS).ok();
            if let Ok(r) = slot.command_blocking(41, ocr_hcs, ResponseLen::Short, false, FLAGS) {
                if r[0] & 0x8000_0000 != 0 {
                    ready = true;
                    break;
                }
            }
            nb_delay_ms(20);
        }
        if !ready {
            return Err("ACMD41 timeout");
        }

        // 提速到 20MHz 运行
        slot.set_bus_low_level(BusWidth::Bit1, 20_000_000).map_err(|_| "bus2")?;

        // CMD2 ALL_SEND_CID
        slot.command_blocking(2, 0, ResponseLen::Long, false, FLAGS).ok();
        // CMD3 SEND_RCA
        let rca = slot
            .command_blocking(3, 0, ResponseLen::Short, false, FLAGS)
            .map(|r| (r[0] >> 16) << 16)
            .map_err(|_| "CMD3")?;
        // CMD7 SELECT
        slot.command_blocking(7, rca, ResponseLen::Short, false, FLAGS).map_err(|_| "CMD7")?;
        // CMD16 SET_BLOCKLEN 512
        slot.command_blocking(16, 512, ResponseLen::Short, false, FLAGS).map_err(|_| "CMD16")?;

        let mut card = SdCard {
            slot,
            part_lba: 0,
            fat: Fat {
                fat32: false,
                sectors_per_cluster: 1,
                fat_start_lba: 0,
                fat_size_sectors: 0,
                num_fats: 2,
                root_dir_lba: 0,
                root_cluster: 2,
                data_start_lba: 0,
                total_sectors: 0,
                cluster_count: 0,
            },
        };

        // 读 MBR / 引导扇区
        let b0 = card.read_block(0)?;
        let (part_lba, total_sectors) = if b0[510] == 0x55 && b0[511] == 0xAA && b0[450] != 0xEB {
            // 有 MBR：取分区 1（type!=0，LBA 起始 + 扇区数）
            let p = &b0[446..462];
            let start = u32::from_le_bytes([p[8], p[9], p[10], p[11]]);
            let count = u32::from_le_bytes([p[12], p[13], p[14], p[15]]);
            (start, count)
        } else {
            (0u32, 0u32) // 超级软盘：BPB 在块 0
        };

        let bs = card.read_block(part_lba)?;
        let fat = card.parse_bpb(&bs, total_sectors)?;
        card.part_lba = part_lba;
        card.fat = fat;
        info!("[sd] mounted FAT{} total={} MB", if card.fat.fat32 { 32 } else { 16 }, card.capacity_mb());
        Ok(card)
    }

    fn parse_bpb(&self, bs: &[u8; 512], mbr_sectors: u32) -> Result<Fat, &'static str> {
        let bytes_per_sector = u16::from_le_bytes([bs[11], bs[12]]) as u32;
        if bytes_per_sector != 512 {
            return Err("non-512 sector");
        }
        let spc = bs[13] as u32;
        let reserved = u16::from_le_bytes([bs[14], bs[15]]) as u32;
        let num_fats = bs[16] as u32;
        let root_entries = u16::from_le_bytes([bs[17], bs[18]]) as u32;
        let mut total16 = u16::from_le_bytes([bs[19], bs[20]]) as u32;
        let spf16 = u16::from_le_bytes([bs[21], bs[22]]) as u32;
        let mut total32 = u32::from_le_bytes([bs[32], bs[33], bs[34], bs[35]]);
        let spf32 = u32::from_le_bytes([bs[36], bs[37], bs[38], bs[39]]);
        let root_cluster = u32::from_le_bytes([bs[44], bs[45], bs[46], bs[47]]);

        let bsig = &bs[82..90];
        if &bsig[4..8] != b"FAT" {
            return Err("no FAT sig");
        }

        let total = if total32 != 0 { total32 } else if total16 != 0 { total16 } else { mbr_sectors };
        total16 = total;
        total32 = total;

        let fat32 = root_entries == 0 && spf16 == 0;
        let fat_size = if fat32 { spf32 } else { spf16 };
        let fat_start = reserved;
        let root_dir_sectors = if fat32 {
            0
        } else {
            (root_entries * 32 + 511) / 512
        };
        let data_start = fat_start + num_fats * fat_size + root_dir_sectors;
        let root_dir_lba = fat_start + num_fats * fat_size;
        let cluster_count = if fat_size * self.fat_clusters_per_sector() > 0 {
            (total - data_start) / spc.max(1)
        } else {
            0
        };

        let fat = Fat {
            fat32,
            sectors_per_cluster: spc,
            fat_start_lba: fat_start,
            fat_size_sectors: fat_size,
            num_fats,
            root_dir_lba,
            root_cluster,
            data_start_lba: data_start,
            total_sectors: total,
            cluster_count,
        };
        Ok(fat)
    }

    fn fat_clusters_per_sector(&self) -> u32 {
        if self.fat.fat32 {
            512 / 4
        } else {
            512 / 2
        }
    }

    fn capacity_mb(&self) -> u32 {
        self.fat.total_sectors * 512 / (1024 * 1024)
    }

    // --------------------------------------------------------
    // 块 / FAT 原语
    // --------------------------------------------------------

    fn read_block(&mut self, lba: u32) -> Result<[u8; 512], &'static str> {
        let mut blk = AlignedBlock([0u8; 512]);
        let mut buf = DmaAlignedMut::new(&mut blk.0[..]).map_err(|_| "align")?;
        self.slot
            .read_blocks_blocking(17, lba, buf.reborrow(), 512, 1)
            .map_err(|_| "read block")?;
        let _ = &mut buf;
        Ok(blk.0)
    }

    fn cluster_to_lba(&self, cluster: u32) -> u32 {
        self.part_lba + self.fat.data_start_lba + (cluster - 2) * self.fat.sectors_per_cluster
    }

    fn read_fat_entry(&mut self, cluster: u32) -> u32 {
        let (off, fat_start) = if self.fat.fat32 {
            (cluster * 4, cluster * 4 / 512)
        } else {
            (cluster * 2, cluster * 2 / 512)
        };
        let off = off as usize;
        let lba = self.part_lba + self.fat.fat_start_lba + fat_start;
        match self.read_block(lba) {
            Ok(b) => {
                if self.fat.fat32 {
                    u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]]) & 0x0FFF_FFFF
                } else {
                    u16::from_le_bytes([b[off], b[off + 1]]) as u32
                }
            }
            Err(_) => 0x0FFF_FFFF,
        }
    }

    fn eoc(cluster: u32, fat32: bool) -> bool {
        if fat32 {
            cluster >= 0x0FFF_FFF8
        } else {
            cluster >= 0xFFF8
        }
    }

    // --------------------------------------------------------
    // 目录 / 文件
    // --------------------------------------------------------

    /// 列出根目录文件：返回 (名称, 字节数)
    pub fn list_files(&mut self) -> Vec<(String, u32)> {
        let mut out = Vec::new();
        if self.fat.fat32 {
            let mut cluster = self.fat.root_cluster;
            while !Self::eoc(cluster, true) && cluster >= 2 {
                self.scan_dir_cluster(cluster, &mut out);
                cluster = self.read_fat_entry(cluster);
            }
        } else {
            let root = self.part_lba + self.fat.root_dir_lba;
            let sectors = (self.fat.root_dir_lba == 0) as usize; // placeholder
            let _ = sectors;
            let root_sectors = ((self.entries_in_root() * 32 + 511) / 512) as u32;
            for s in 0..root_sectors {
                if let Ok(b) = self.read_block(root + s) {
                    self.scan_dir_block(&b, &mut out);
                }
            }
        }
        out
    }

    fn entries_in_root(&self) -> usize {
        // FAT16 根目录条目数：由 root_dir 区大小反推（最多 512 条）
        512
    }

    fn scan_dir_cluster(&mut self, cluster: u32, out: &mut Vec<(String, u32)>) {
        let lba = self.cluster_to_lba(cluster);
        for s in 0..self.fat.sectors_per_cluster {
            if let Ok(b) = self.read_block(lba + s) {
                self.scan_dir_block(&b, out);
            }
        }
    }

    fn scan_dir_block(&mut self, b: &[u8; 512], out: &mut Vec<(String, u32)>) {
        let mut lfn: String = String::new();
        for e in b.chunks_exact(32) {
            let attr = e[11];
            if e[0] == 0x00 {
                break; // 空槽，后面无条目
            }
            if e[0] == 0xE5 {
                continue; // 已删除
            }
            if attr == 0x0F {
                // 长文件名片段
                let part = parse_lfn(e);
                lfn = part + &lfn;
                continue;
            }
            if attr & 0x08 != 0 {
                continue; // VOLUME_ID
            }
            let name = if !lfn.is_empty() {
                core::mem::take(&mut lfn)
            } else {
                short_name(e)
            };
            let is_dir = attr & 0x10 != 0;
            let size = u32::from_le_bytes([e[28], e[29], e[30], e[31]]);
            if !is_dir {
                out.push((name, size));
            }
        }
    }

    /// 读取根目录下的文件内容（按短名或长名匹配，大小写不敏感）
    pub fn read_file(&mut self, want: &str) -> Option<Vec<u8>> {
        let files = self.list_files();
        // 找到目标文件的首簇：重新遍历目录取首簇
        let cluster = self.find_file_cluster(want, &files)?;
        let size = files
            .iter()
            .find(|(n, _)| n.eq_ignore_ascii_case(want))
            .map(|(_, s)| *s)?;
        let mut data: Vec<u8> = vec![0u8; size as usize];
        let mut c = cluster;
        let mut pos = 0usize;
        while pos < size as usize && c >= 2 && !Self::eoc(c, self.fat.fat32) {
            let lba = self.cluster_to_lba(c);
            for s in 0..self.fat.sectors_per_cluster {
                if pos >= size as usize {
                    break;
                }
                let blk = self.read_block(lba + s).ok()?;
                let n = core::cmp::min(512, size as usize - pos);
                data[pos..pos + n].copy_from_slice(&blk[..n]);
                pos += n;
            }
            c = self.read_fat_entry(c);
        }
        Some(data)
    }

    /// 扫描一个目录数据块，命中文件返回其首簇
    fn scan_block_for_file(b: &[u8; 512], lfn: &mut String, want: &str, fat32: bool) -> Option<u32> {
        for e in b.chunks_exact(32) {
            if e[0] == 0x00 {
                break;
            }
            if e[0] == 0xE5 {
                continue;
            }
            if e[11] == 0x0F {
                *lfn = parse_lfn(e) + &*lfn;
                continue;
            }
            let name = if !lfn.is_empty() {
                core::mem::take(lfn)
            } else {
                short_name(e)
            };
            if name.eq_ignore_ascii_case(want) {
                let hi = u16::from_le_bytes([e[20], e[21]]) as u32;
                let lo = u16::from_le_bytes([e[26], e[27]]) as u32;
                return Some(if fat32 { (hi << 16) | lo } else { lo });
            }
        }
        None
    }

    fn find_file_cluster(&mut self, want: &str, _files: &[(String, u32)]) -> Option<u32> {
        let fat32 = self.fat.fat32;
        let mut lfn = String::new();
        if fat32 {
            let mut c = self.fat.root_cluster;
            while !Self::eoc(c, true) && c >= 2 {
                let lba = self.cluster_to_lba(c);
                for s in 0..self.fat.sectors_per_cluster {
                    let b = self.read_block(lba + s).ok()?;
                    if let Some(x) = Self::scan_block_for_file(&b, &mut lfn, want, true) {
                        return Some(x);
                    }
                }
                c = self.read_fat_entry(c);
            }
            None
        } else {
            let root = self.part_lba + self.fat.root_dir_lba;
            let root_sectors = ((self.entries_in_root() * 32 + 511) / 512) as u32;
            for s in 0..root_sectors {
                let b = self.read_block(root + s).ok()?;
                if let Some(x) = Self::scan_block_for_file(&b, &mut lfn, want, false) {
                    return Some(x);
                }
            }
            None
        }
    }
}

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

fn parse_lfn(e: &[u8]) -> String {
    let mut s = String::new();
    let idx = [1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24];
    for &i in &idx {
        let u = u16::from_le_bytes([e[i], e[i + 1]]);
        if u == 0xFFFF {
            continue;
        }
        if u == 0x0000 {
            break;
        }
        if let Some(c) = char::from_u32(u as u32) {
            s.push(c);
        }
    }
    s
}

// 简单忙等延时（初始化阶段用，毫秒级；此时任务尚未并发）
fn nb_delay_ms(ms: u32) {
    let delay = esp_hal::delay::Delay::new();
    delay.delay_millis(ms);
}

// ------------------------------------------------------------
// 全局访问（供 status / shell / web）
// ------------------------------------------------------------

/// 由 main 在拿到外设后调用一次：尝试初始化并挂载。
pub fn try_mount(
    sdhost: SDHOST<'static>,
    clk: esp_hal::peripherals::GPIO39<'static>,
    cmd: esp_hal::peripherals::GPIO38<'static>,
    data0: esp_hal::peripherals::GPIO40<'static>,
) {
    match SdCard::init(sdhost, clk, cmd, data0) {
        Ok(card) => unsafe {
            CARD = Some(card);
        },
        Err(e) => {
            warn!("[sd] mount failed: {}", e);
            unsafe {
                CARD = None;
            }
        }
    }
}

/// 通过裸指针访问全局，规避 Rust 2024 的 static_mut_refs。
/// 单线程协作式执行器下 SD 操作均为同步无 await，不会重入。
fn card_ref() -> Option<&'static SdCard> {
    unsafe { (*core::ptr::addr_of!(CARD)).as_ref() }
}

fn card_mut() -> Option<&'static mut SdCard> {
    unsafe { (*core::ptr::addr_of_mut!(CARD)).as_mut() }
}

pub fn info() -> SdInfo {
    match card_ref() {
        Some(c) => SdInfo {
            present: true,
            mounted: true,
            fat_type: if c.fat.fat32 { 32 } else { 16 },
            total_mb: c.capacity_mb(),
            free_mb: 0,
        },
        None => SdInfo {
            present: false,
            mounted: false,
            fat_type: 0,
            total_mb: 0,
            free_mb: 0,
        },
    }
}

pub fn list_files() -> Vec<(String, u32)> {
    match card_mut() {
        Some(c) => c.list_files(),
        None => Vec::new(),
    }
}

pub fn read_file(name: &str) -> Option<Vec<u8>> {
    match card_mut() {
        Some(c) => c.read_file(name),
        None => None,
    }
}
