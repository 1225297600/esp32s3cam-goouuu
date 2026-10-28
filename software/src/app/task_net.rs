use core::cell::RefCell;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_net::{Config as NetConfig, Runner, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Interface, WifiController,
};
use static_cell::StaticCell;
use crate::app::task_shell::task_shell;
use crate::app::task_web::{task_web, WEB_TASK_POOL_SIZE};

const WIFI_NAME: &str = "rayrobot";
const WIFI_PASSWORD: &str = "Rayrobot123";

/// 网络信息快照（供 /api/status）
#[derive(Clone, Copy)]
pub struct NetInfo {
    pub ip: [u8; 4],
    pub prefix_len: u8,
    pub gateway: [u8; 4],
}

pub static NET_INFO: Mutex<CriticalSectionRawMutex, RefCell<NetInfo>> =
    Mutex::new(RefCell::new(NetInfo {
        ip: [0; 4],
        prefix_len: 0,
        gateway: [0; 4],
    }));

// 网络栈资源（静态存储，保证 'static 生命周期）
static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();

#[embassy_executor::task]
pub async fn task_net(
    spawner: Spawner,
    wifi_peripheral: esp_hal::peripherals::WIFI<'static>,
) -> ! {
    info!("[net] task starting");

    // ---- Wi-Fi 初始化 (STA 模式) ----
    let station_config = StationConfig::default()
        .with_ssid(WIFI_NAME.try_into().unwrap())
        .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
            WIFI_PASSWORD.try_into().unwrap(),
        ));

    let controller_config = ControllerConfig::default()
        .with_initial_config(Config::Station(station_config));

    let mut wifi_controller = WifiController::new(wifi_peripheral, controller_config)
        .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = Interface::station();

    // ---- 连接 Wi-Fi (链路层) ----
    info!("[net] connecting to Wi-Fi...");
    match wifi_controller.connect_async().await {
        Ok(info) => info!("[net] Wi-Fi connected: {:?}", info),
        Err(e) => {
            error!("[net] Wi-Fi connect failed: {:?}", e);
            panic!("Wi-Fi connect failed");
        }
    }

    // ---- 初始化 embassy-net 网络栈 ----
    let net_config = NetConfig::dhcpv4(Default::default());
    let resources = RESOURCES.init(StackResources::new());

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        resources,
        0x0123_4567_89ab_cdef,
    );

    // ---- 启动网络栈运行器 ----
    spawner.spawn(net_runner(runner).unwrap());
    info!("[net] net_runner spawned");

    // ---- 等待 DHCP 完成 ----
    info!("[net] waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        info!("[net] IPv4 address: {}", config.address);
        NET_INFO.lock(|n| {
            *n.borrow_mut() = NetInfo {
                ip: config.address.address().octets(),
                prefix_len: config.address.prefix_len(),
                gateway: config.gateway.map(|g| g.octets()).unwrap_or([0; 4]),
            };
        });
    } else {
        error!("[net] no IP address obtained");
    }

    // ---- 直接把 Stack 传给 web 任务（Stack 是 Copy）----
    for task_id in 0..WEB_TASK_POOL_SIZE {
        spawner.spawn(task_web(task_id, stack).unwrap());
    }
    info!("[net] web task spawned with stack");

    // ---- TCP shell (telnet, 端口 23) ----
    spawner.spawn(task_shell(stack).unwrap());
    info!("[net] shell task spawned on tcp/23");

    // wifi_controller 必须保活，否则 Wi-Fi 连接会断开
    let _keep_alive = wifi_controller;

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn net_runner(mut runner: Runner<'static, Interface>) {
    runner.run().await
}