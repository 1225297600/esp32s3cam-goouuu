// src/app/task_web.rs
use defmt::info;
use embassy_net::Stack;
use embassy_time::{Duration, Timer};
use picoserve::routing::{get, post, parse_path_segment};

use crate::app::routes::{
    api_camera_params_get, api_camera_params_set, api_capture, api_sd_download, api_sd_list,
    api_status, api_ws2812, index, stream,
};

pub const WEB_TASK_POOL_SIZE: usize = 2;

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
pub async fn task_web(task_id: usize, stack: Stack<'static>) -> ! {
    info!("[web] task {} starting", task_id);

    let app = picoserve::Router::new()
        .route("/", get(index))
        .route("/stream", get(stream))
        .route("/api/status", get(api_status))
        .route("/api/capture", get(api_capture))
        .route("/api/sd/list", get(api_sd_list))
        .route(
            ("/api/sd/download", parse_path_segment::<heapless::String<64>>()),
            get(api_sd_download),
        )
        .route("/api/camera/params", get(api_camera_params_get))
        .route(
            (
                "/api/camera/params",
                parse_path_segment::<u8>(),
                parse_path_segment::<u8>(),
            ),
            post(api_camera_params_set),
        )
        .route(
            (
                "/api/ws2812",
                parse_path_segment::<u8>(),
                parse_path_segment::<u8>(),
                parse_path_segment::<u8>(),
            ),
            post(api_ws2812),
        );

    let config = picoserve::Config::const_default().keep_connection_alive();

    // 为共享的 main-task 执行器栈保留余量，缓冲区取保守值
    let mut tcp_rx_buffer = [0u8; 1024];
    let mut tcp_tx_buffer = [0u8; 2048];
    let mut http_buffer = [0u8; 2048];

    if let Some(net) = stack.config_v4() {
        info!("[web] visit http://{}", net.address.address());
    }

    picoserve::Server::new(&app, &config, &mut http_buffer)
        .listen_and_serve(task_id, stack, 80, &mut tcp_rx_buffer, &mut tcp_tx_buffer)
        .await;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
