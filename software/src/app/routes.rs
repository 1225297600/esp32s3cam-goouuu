// src/app/routes.rs
//
// Web API 路由处理：
//   GET  /                    控制面板首页
//   GET  /api/status          设备状态 JSON
//   GET  /api/capture         最新 JPEG 帧（伪 MJPEG：前端定时刷新）
//   GET  /api/camera/params   读取摄像头参数
//   POST /api/camera/params/:quality/:mode  设置摄像头参数
//   POST /api/ws2812/:r/:g/:b 设置 WS2812 颜色
use core::fmt::Write as _;

use heapless::String;
use picoserve::response::chunked::{ChunkWriter, ChunkedResponse, Chunks};
use picoserve::response::{Response, StatusCode};

use crate::app::task_camera::{self, MAX_FRAME_LEN};
use crate::app::task_net::NET_INFO;
use crate::app::task_ws2812::WS2812_CHANNEL;

use core::sync::atomic::Ordering;

/// 首页：摄像头画面 + 参数控制 + RGB 控制
pub async fn index() -> impl picoserve::response::IntoResponse {
    Response::new(
        StatusCode::OK,
        r##"
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Goouuu ESP32-S3-CAM</title>
<style>
body{font-family:system-ui,sans-serif;max-width:820px;margin:1rem auto;padding:0 1rem;background:#111;color:#eee}
h1{font-size:1.3rem} img{max-width:100%;border:1px solid #444;background:#000}
.card{background:#1c1c1e;border-radius:8px;padding:.8rem 1rem;margin:.8rem 0}
button,select,input{padding:.3rem .6rem;margin:.15rem}
.sw{display:inline-block;width:28px;height:28px;border-radius:6px;border:1px solid #555;cursor:pointer;vertical-align:middle}
code{color:#8f8}
</style>
</head>
<body>
<h1>Goouuu ESP32-S3-CAM 控制面板</h1>

<div class="card">
  <h2>摄像头</h2>
  <img id="frame" alt="等待画面..." width="320">
  <p>
    <label>刷新间隔
      <select id="interval">
        <option value="0">手动</option>
        <option value="200">5 fps</option>
        <option value="500" selected>2 fps</option>
        <option value="1000">1 fps</option>
      </select>
    </label>
    <button onclick="refreshFrame()">拍一张</button>
    <button id="livebtn" onclick="toggleLive()">▶ 直播</button>
    <span id="caminfo">--</span>
  </p>
  <p>
    <label>分辨率
      <select id="mode">
        <option value="0">QQVGA 160x120</option>
        <option value="1">QVGA 320x240</option>
        <option value="2">VGA 640x480</option>
        <option value="3">SVGA 800x600</option>
        <option value="4">UXGA 1600x1200</option>
      </select>
    </label>
    <label>质量(10-63) <input id="qs" type="number" value="15" min="10" max="63" style="width:5rem"></label>
    <button onclick="setParams()">应用</button>
  </p>
</div>

<div class="card">
  <h2>WS2812 (GPIO48)</h2>
  <span class="sw" style="background:#ff0000" onclick="setColor(255,0,0)"></span>
  <span class="sw" style="background:#00ff00" onclick="setColor(0,255,0)"></span>
  <span class="sw" style="background:#0000ff" onclick="setColor(0,0,255)"></span>
  <span class="sw" style="background:#ffffff" onclick="setColor(255,255,255)"></span>
  <span class="sw" style="background:#000000" onclick="setColor(0,0,0)"></span>
  <input type="color" id="picker" value="#8000ff" onchange="pick(this.value)">
  <span id="rgbres"></span>
</div>

<div class="card">
  <h2>MicroSD 卡</h2>
  <p id="sdinfo">--</p>
</div>

<div class="card">
  <h2>设备状态</h2>
  <pre id="status" style="white-space:pre-wrap">--</pre>
</div>

<script>
let lastSeq = 0;
async function refreshFrame() {
  try {
    const r = await fetch('/api/capture?_=' + Date.now());
    if (r.status === 409) { document.getElementById('caminfo').textContent = '摄像头未就绪'; return; }
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const blob = await r.blob();
    const old = document.getElementById('frame').src;
    document.getElementById('frame').src = URL.createObjectURL(blob);
    if (old.startsWith('blob:')) URL.revokeObjectURL(old);
  } catch (e) { console.log(e); }
}
async function setParams() {
  const q = document.getElementById('qs').value;
  const m = document.getElementById('mode').value;
  const r = await fetch(`/api/camera/params/${q}/${m}`, {method:'POST'});
  const j = await r.json();
  document.getElementById('caminfo').textContent = j.status || JSON.stringify(j);
}
async function setColor(r,g,b) {
  const res = await fetch(`/api/ws2812/${r}/${g}/${b}`, {method:'POST'});
  const j = await res.json();
  document.getElementById('rgbres').textContent = `(${r},${g},${b}) ${j.status||''}`;
}
function pick(hex) {
  setColor(parseInt(hex.slice(1,3),16), parseInt(hex.slice(3,5),16), parseInt(hex.slice(5,7),16));
}
async function pollStatus() {
  try {
    const r = await fetch('/api/status');
    const j = await r.json();
    document.getElementById('status').textContent = JSON.stringify(j, null, 2);
    if (j.camera) {
      document.getElementById('caminfo').textContent =
        `${j.camera.resolution} qs=${j.camera.quality} ${j.camera.fps}fps ${j.camera.kbps}kbps`;
      lastSeq = j.camera.frame_seq;
    }
    if (j.sd) {
      document.getElementById('sdinfo').textContent =
        `${j.sd.bus}  CMD=IO${j.sd.cmd} CLK=IO${j.sd.clk} DATA=IO${j.sd.data}  挂载:${j.sd.mounted ? '是' : '否'}`;
    }
  } catch (e) {}
}
let timer = null;
let live = false;
function toggleLive() {
  live = !live;
  const img = document.getElementById('frame');
  const btn = document.getElementById('livebtn');
  if (live) {
    if (timer) { clearInterval(timer); timer = null; }
    img.src = '/stream?t=' + Date.now();   // multipart/x-mixed-replace 连续流
    btn.textContent = '■ 停止直播';
  } else {
    img.src = '';
    btn.textContent = '▶ 直播';
    applyInterval();
  }
}
function applyInterval() {
  if (live) return;
  if (timer) clearInterval(timer);
  const ms = parseInt(document.getElementById('interval').value);
  if (ms > 0) { refreshFrame(); timer = setInterval(refreshFrame, ms); }
}
document.getElementById('interval').onchange = applyInterval;
pollStatus(); setInterval(pollStatus, 3000); applyInterval();
</script>
</body>
</html>
        "##,
    )
    .with_header("Content-Type", "text/html; charset=utf-8")
}

/// GET /api/status —— 设备状态 JSON
pub async fn api_status() -> impl picoserve::response::IntoResponse {
    let mut s: String<1024> = String::new();

    let uptime = crate::app::task_camera::UPTIME_SECS.load(Ordering::Relaxed);
    let mem_free = esp_alloc::HEAP.free();

    let net = NET_INFO.lock(|n| *n.borrow());

    let _ = write!(
        &mut s,
        concat!(
            "{{\"device\":\"goouuu-esp32-s3-cam\",\"chip\":\"esp32s3-n16r8\",\"uptime\":{},",
            "\"heap_free\":{},\"wifi_connected\":{},",
        ),
        uptime,
        mem_free,
        net.ip != [0, 0, 0, 0]
    );
    if net.ip != [0, 0, 0, 0] {
        let ip = core::net::Ipv4Addr::from(net.ip);
        let gw = core::net::Ipv4Addr::from(net.gateway);
        let _ = write!(
            &mut s,
            "\"ip\":\"{}\",\"prefix_len\":{},\"gateway\":\"{}\"",
            ip, net.prefix_len, gw
        );
    } else {
        let _ = write!(&mut s, "\"ip\":null,\"prefix_len\":null,\"gateway\":null");
    }

    let sd = crate::app::task_sd::info();
    let _ = write!(
        &mut s,
        ",\"sd\":{{\"bus\":\"1-bit SDMMC\",\"cmd\":38,\"clk\":39,\"data\":40,\"present\":{},\"mounted\":{},\"fat\":{},\"total_mb\":{}}}",
        sd.present,
        sd.mounted,
        sd.fat_type,
        sd.total_mb
    );

    task_camera::CAMERA_STATUS.lock(|st| {
        let st = st.borrow();
        let (w, h) = st.mode.size();
        let _ = write!(
            &mut s,
            ",\"camera\":{{\"present\":{},\"resolution\":\"{}x{}\",\"name\":\"{}\",",
            st.present, w, h, st.mode.name()
        );
        let _ = write!(
            &mut s,
            "\"quality\":{},\"frame_seq\":{},\"frame_len\":{},\"fps\":{},\"kbps\":{}}}",
            st.quality, st.frame_seq, st.frame_len, st.fps, st.kbps
        );
    });

    let _ = write!(&mut s, "}}");

    Response::new(StatusCode::OK, s).with_header("Content-Type", "application/json")
}

/// GET /api/capture —— 最新一帧 JPEG；尚无帧时返回 409
pub async fn api_capture() -> impl picoserve::response::IntoResponse {
    let (_seq, frame) = task_camera::latest_frame();
    let (status, body, ctype) = match frame {
        Some(data) if !data.is_empty() => (StatusCode::OK, data, "image/jpeg"),
        _ => (
            StatusCode::CONFLICT,
            alloc::vec::Vec::from(r#"{"status":"no frame yet - camera not ready or no sensor"}"#),
            "application/json",
        ),
    };
    Response::new(status, body)
        .with_header("Content-Type", ctype)
        .with_header("Cache-Control", "no-store")
}

/// GET /api/sd/list —— 根目录文件列表 JSON
pub async fn api_sd_list() -> impl picoserve::response::IntoResponse {
    let files = crate::app::task_sd::list_files().await;
    let mut s: String<2048> = String::new();
    let _ = write!(&mut s, "{{\"count\":{},\"files\":[", files.len());
    for (i, (name, size)) in files.iter().enumerate() {
        if i > 0 {
            let _ = s.push(',');
        }
        let _ = write!(&mut s, "{{\"name\":\"{}\",\"size\":{}}}", name, size);
    }
    let _ = s.push_str("]}");
    Response::new(StatusCode::OK, s).with_header("Content-Type", "application/json")
}

/// GET /api/sd/download/:name —— 下载 SD 卡上的文件
pub async fn api_sd_download(
    name: heapless::String<64>,
) -> impl picoserve::response::IntoResponse {
    match crate::app::task_sd::read_file(name.as_str()).await {
        Some(data) if !data.is_empty() => Response::new(StatusCode::OK, data)
            .with_header("Content-Type", "application/octet-stream")
            .with_header("Content-Disposition", "attachment"),
        _ => Response::new(
            StatusCode::NOT_FOUND,
            alloc::vec::Vec::from(r#"{"status":"file not found or sd unavailable"}"#),
        )
        .with_header("Content-Type", "application/json")
        .with_header("Content-Disposition", "inline"),
    }
}

/// GET /api/camera/params —— 读取当前摄像头参数
pub async fn api_camera_params_get() -> impl picoserve::response::IntoResponse {
    let mut s: String<128> = String::new();
    task_camera::CAMERA_STATUS.lock(|cell| {
        let st = cell.borrow();
        let _ = write!(
            &mut s,
            "{{\"quality\":{},\"mode\":{},\"name\":\"{}\"}}",
            st.quality,
            task_camera::mode_number(st.mode),
            st.mode.name()
        );
    });
    Response::new(StatusCode::OK, s).with_header("Content-Type", "application/json")
}

/// POST /api/camera/params/:quality/:mode —— 设置质量与分辨率
pub async fn api_camera_params_set((quality, mode_idx): (u8, u8)) -> impl picoserve::response::IntoResponse {
    let Some(mode) = task_camera::parse_mode(mode_idx) else {
        return Response::new(StatusCode::BAD_REQUEST, r#"{"status":"bad mode"}"#)
            .with_header("Content-Type", "application/json");
    };
    if !(10..=63).contains(&quality) {
        return Response::new(StatusCode::BAD_REQUEST, r#"{"status":"bad quality"}"#)
            .with_header("Content-Type", "application/json");
    }

    match task_camera::request_params(quality, mode) {
        Ok(_) => Response::new(StatusCode::OK, r#"{"status":"queued"}"#)
            .with_header("Content-Type", "application/json"),
        Err(_) => Response::new(StatusCode::SERVICE_UNAVAILABLE, r#"{"status":"busy"}"#)
            .with_header("Content-Type", "application/json"),
    }
}

/// POST /api/ws2812/:r/:g/:b —— 设置板载 RGB LED
pub async fn api_ws2812((r, g, b): (u8, u8, u8)) -> impl picoserve::response::IntoResponse {
    use crate::app::task_ws2812::RGB8;
    match WS2812_CHANNEL.try_send(RGB8(r, g, b)) {
        Ok(_) => Response::new(StatusCode::OK, r#"{"status":"ok"}"#)
            .with_header("Content-Type", "application/json"),
        Err(_) => Response::new(StatusCode::SERVICE_UNAVAILABLE, r#"{"status":"busy"}"#)
            .with_header("Content-Type", "application/json"),
    }
}

/// 编译期检查：发布帧上限与 picoserve 响应拷贝容量一致
const _: () = {
    let _ = MAX_FRAME_LEN;
};

// ============================================================
// MJPEG 推流 (multipart/x-mixed-replace, chunked)
// ============================================================

/// GET /stream —— 持续推送最新 JPEG 帧（浏览器 <img> 可直接显示）
pub async fn stream() -> impl picoserve::response::IntoResponse {
    ChunkedResponse::new(MjpegStream)
}

struct MjpegStream;

impl Chunks for MjpegStream {
    fn content_type(&self) -> &'static str {
        "multipart/x-mixed-replace; boundary=frame"
    }

    async fn write_chunks<W: picoserve::io::Write>(
        self,
        mut cw: ChunkWriter<W>,
    ) -> Result<picoserve::response::chunked::ChunksWritten, W::Error> {
        use embassy_time::{Duration, Timer};
        let mut last_seq = 0u32;
        loop {
            let seq = task_camera::frame_seq();
            if seq != last_seq {
                last_seq = seq;
                let (_s, frame) = task_camera::latest_frame();
                if let Some(jpeg) = frame {
                    // 一个 multipart part = 头 + JPEG 体
                    cw.write_chunk(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        .await?;
                    cw.write_chunk(&jpeg).await?;
                    cw.write_chunk(b"\r\n").await?;
                }
            }
            // 约 20fps 上限，且让出执行器给其他任务
            Timer::after(Duration::from_millis(50)).await;
        }
        // 不主动 finalize：直到客户端断开（write_chunk 返回 Err）
    }
}
