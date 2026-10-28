// src/app/task_shell.rs
//
// 基于 embedded-cli 框架的 TCP shell（端口 23）。
// embedded-cli 提供命令解析/参数类型检查/help/Tab 补全/历史回溯，
// 本文件只做两件事：
//   1) 字节桥接：async TcpSocket 的收发字节喂给 cli.process_byte，
//      框架输出经 BufWriter 收集到静态 Vec，再批量 flush 回 socket；
//   2) 命令实现：用 #[derive(Command)] 声明的强类型命令表。
use alloc::vec::Vec;
use defmt::info;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embedded_cli::cli::CliBuilder;
use embedded_cli::Command;
use embedded_io06::Write as Write06;
use embedded_io_async::Write;
use heapless::String;

use crate::app::task_camera::{self, parse_mode};
use crate::app::task_net::NET_INFO;
use crate::app::task_ws2812::{RGB8, WS2812_CHANNEL};

const PORT: u16 = 23;

/// 桥接错误类型（embedded-io 0.6）
#[derive(Debug)]
pub struct IoErr;
impl core::fmt::Display for IoErr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("io")
    }
}
impl embedded_io06::Error for IoErr {
    fn kind(&self) -> embedded_io06::ErrorKind {
        embedded_io06::ErrorKind::Other
    }
}

/// 输出收集器：embedded-cli 写入的字节先进静态 Vec，由外层 flush 到 socket
struct CliOut;
impl embedded_io06::ErrorType for CliOut {
    type Error = IoErr;
}
impl Write06 for CliOut {
    fn write(&mut self, buf: &[u8]) -> Result<usize, IoErr> {
        push_out(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), IoErr> {
        Ok(())
    }
}

static mut OUT: Vec<u8> = Vec::new();

/// 追加原始字节到输出缓冲（单线程执行器内串行使用）
fn push_out(b: &[u8]) {
    unsafe { (*core::ptr::addr_of_mut!(OUT)).extend_from_slice(b) };
}

/// 格式化输出到输出缓冲
fn pout(args: core::fmt::Arguments) {
    let mut s: String<512> = String::new();
    let _ = core::fmt::write(&mut s, args);
    push_out(s.as_bytes());
}

fn take_out() -> Vec<u8> {
    unsafe { core::mem::take(&mut *core::ptr::addr_of_mut!(OUT)) }
}

/// 命令表：doc 注释即 help 文本；参数为强类型（u8/&str）
#[derive(Command)]
enum Shell<'a> {
    /// Show device status (json)
    Status,

    /// Show free heap bytes
    Heap,

    /// Show uptime seconds
    Uptime,

    /// Show network info
    Wifi,

    /// Show camera status
    Camera,

    /// Set resolution: 0=QQVGA 1=QVGA 2=VGA 3=SVGA 4=UXGA
    Resolution {
        /// mode 0-4
        mode: u8,
    },

    /// Set JPEG quality (quantizer 10-63, smaller = better)
    Quality {
        /// qs 10-63
        qs: u8,
    },

    /// Set WS2812 color, e.g. led 255 0 0
    Led {
        /// red 0-255
        r: u8,
        /// green 0-255
        g: u8,
        /// blue 0-255
        b: u8,
    },

    /// Show latest frame seq/size
    Capture,

    /// Show SD card status
    Sd,

    /// List files on SD card
    Sdls,

    /// Read a file from SD card
    Sdget {
        /// file name
        name: &'a str,
    },

    /// Close this connection
    Exit,
}

#[embassy_executor::task]
pub async fn task_shell(stack: Stack<'static>) -> ! {
    info!("[shell] embedded-cli listening on tcp/{}", PORT);
    loop {
        let mut rx = [0u8; 512];
        let mut tx = [0u8; 1024];
        let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);

        if socket.accept(PORT).await.is_err() {
            continue;
        }

        // embedded-cli 的输入/历史缓冲区（静态，避免占栈）
        let cmd_buf = unsafe { &mut *core::ptr::addr_of_mut!(CMD_BUF) };
        let cmd_buf: &mut [u8] = &mut cmd_buf[..];
        let hist_buf = unsafe { &mut *core::ptr::addr_of_mut!(HIST_BUF) };
        let hist_buf: &mut [u8] = &mut hist_buf[..];
        let mut cli = match CliBuilder::default()
            .writer(CliOut)
            .command_buffer(cmd_buf)
            .history_buffer(hist_buf)
            .prompt("> ")
            .build()
        {
            Ok(c) => c,
            Err(_) => {
                socket.close();
                continue;
            }
        };

        let banner = b"Goouuu ESP32-S3-CAM shell (embedded-cli)\r\n\
                      type 'help' for commands, Tab completes, Up/Down history\r\n";
        if socket.write_all(banner).await.is_err() {
            continue;
        }

        let mut running = true;
        let mut buf = [0u8; 64];
        while running {
            match socket.read(&mut buf).await {
                Ok(0) | Err(_) => break,
                Ok(n) => {
                    for &b in &buf[..n] {
                        let _ = cli.process_byte::<Shell, _>(
                            b,
                            &mut Shell::processor(|_cli, cmd| {
                                match cmd {
                                    Shell::Status => {
                                        let (wd, ht) = task_camera::CAMERA_STATUS.lock(|s| {
                                            let s = s.borrow();
                                            s.mode.size()
                                        });
                                        let seq = task_camera::frame_seq();
                                        let free = esp_alloc::HEAP.free();
                                        let up = task_camera::UPTIME_SECS
                                            .load(core::sync::atomic::Ordering::Relaxed);
                                        pout(format_args!(
                                            "{{\"heap_free\":{},\"uptime\":{},\"frame_seq\":{},\"res\":\"{}x{}\"}}\r\n",
                                            free, up, seq, wd, ht
                                        ));
                                    }
                                    Shell::Heap => {
                                        pout(format_args!("free = {} bytes\r\n", esp_alloc::HEAP.free()));
                                    }
                                    Shell::Uptime => {
                                        let up = task_camera::UPTIME_SECS
                                            .load(core::sync::atomic::Ordering::Relaxed);
                                        pout(format_args!("uptime = {} s\r\n", up));
                                    }
                                    Shell::Wifi => {
                                        let net = NET_INFO.lock(|n| *n.borrow());
                                        if net.ip != [0, 0, 0, 0] {
                                            let ip = core::net::Ipv4Addr::from(net.ip);
                                            let gw = core::net::Ipv4Addr::from(net.gateway);
                                            pout(format_args!(
                                                "ip={} prefix/{} gw={}\r\n",
                                                ip, net.prefix_len, gw
                                            ));
                                        } else {
                                            push_out(b"wifi: not connected\r\n");
                                        }
                                    }
                                    Shell::Camera => {
                                        task_camera::CAMERA_STATUS.lock(|s| {
                                            let st = s.borrow();
                                            let (wd, ht) = st.mode.size();
                                            pout(format_args!(
                                                "present={} res={}x{}({}) qs={} fps={} kbps={} seq={}\r\n",
                                                st.present,
                                                wd,
                                                ht,
                                                st.mode.name(),
                                                st.quality,
                                                st.fps,
                                                st.kbps,
                                                st.frame_seq
                                            ));
                                        });
                                    }
                                    Shell::Resolution { mode } => {
                                        let cur_q =
                                            task_camera::CAMERA_STATUS.lock(|s| s.borrow().quality);
                                        match parse_mode(mode) {
                                            Some(m)
                                                if task_camera::request_params(cur_q, m).is_ok() =>
                                            {
                                                push_out(b"resolution queued\r\n")
                                            }
                                            Some(_) => push_out(b"busy, retry\r\n"),
                                            None => push_out(b"usage: resolution <0-4>\r\n"),
                                        }
                                    }
                                    Shell::Quality { qs } => {
                                        let cur =
                                            task_camera::CAMERA_STATUS.lock(|s| s.borrow().mode);
                                        if (10..=63).contains(&qs)
                                            && task_camera::request_params(qs, cur).is_ok()
                                        {
                                            push_out(b"quality queued\r\n")
                                        } else {
                                            push_out(b"usage: quality <10-63> (or busy)\r\n")
                                        }
                                    }
                                    Shell::Led { r, g, b } => {
                                        if WS2812_CHANNEL.try_send(RGB8(r, g, b)).is_ok() {
                                            push_out(b"led set\r\n")
                                        } else {
                                            push_out(b"busy, retry\r\n")
                                        }
                                    }
                                    Shell::Capture => {
                                        let (seq, frame) = task_camera::latest_frame();
                                        let len = frame.map(|v| v.len()).unwrap_or(0);
                                        pout(format_args!("frame_seq={} size={} bytes\r\n", seq, len));
                                    }
                                    Shell::Sd => {
                                        let sd = crate::app::task_sd::info();
                                        pout(format_args!(
                                            "present={} mounted={} fat={} total={} MB\r\n",
                                            sd.present, sd.mounted, sd.fat_type, sd.total_mb
                                        ));
                                    }
                                    Shell::Sdls => {
                                        let files = crate::app::task_sd::list_files();
                                        if files.is_empty() {
                                            push_out(b"(no files or not mounted)\r\n");
                                        }
                                        for (name, size) in files {
                                            pout(format_args!("{}  {} bytes\r\n", name, size));
                                        }
                                    }
                                    Shell::Sdget { name } => {
                                        match crate::app::task_sd::read_file(name) {
                                            Some(data) => pout(format_args!(
                                                "{}: {} bytes\r\n",
                                                name,
                                                data.len()
                                            )),
                                            None => push_out(b"read failed or not found\r\n"),
                                        }
                                    }
                                    Shell::Exit => {
                                        running = false;
                                        push_out(b"bye\r\n");
                                    }
                                }
                                Ok(())
                            }),
                        );
                    }
                    let out = take_out();
                    if !out.is_empty() {
                        if socket.write_all(&out).await.is_err() {
                            break;
                        }
                        let _ = socket.flush().await;
                    }
                }
            }
        }

        socket.close();
    }
}

static mut CMD_BUF: [u8; 64] = [0; 64];
static mut HIST_BUF: [u8; 256] = [0; 256];
