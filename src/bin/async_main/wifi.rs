use core::net::Ipv4Addr;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Runner, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::{RADIO_CLK, RNG, TIMG0, WIFI};
use esp_hal::rng::Rng;
use esp_wifi::wifi::{
    AccessPointConfiguration, ClientConfiguration, Configuration, WifiController, WifiDevice,
    WifiEvent, WifiState,
};
use esp_wifi::{init, EspWifiController};
use log::{error, info};
use smoltcp::wire::Ipv4Cidr;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

pub async fn wifi_init<'a>(
    spawner: Spawner,
    timg0: TIMG0,
    rng: RNG,
    radio_clk: RADIO_CLK,
    wifi: WIFI,
    ap_server_rx_buffer: &'a mut [u8; 1536],
    ap_server_tx_buffer: &'a mut [u8; 1536],
    sta_server_rx_buffer: &'a mut [u8; 1536],
    sta_server_tx_buffer: &'a mut [u8; 1536],
) -> (TcpSocket<'a>, TcpSocket<'a>) {
    esp_alloc::heap_allocator!(72 * 1024);

    let timer_group1 = esp_hal::timer::timg::TimerGroup::new(timg0);
    let mut rng = Rng::new(rng);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timer_group1.timer0, rng, radio_clk).unwrap()
    );

    let (wifi_ap_device, wifi_sta_device, mut controller) =
        esp_wifi::wifi::new_ap_sta(esp_wifi_ctrl, wifi).unwrap();

    let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Addr::new(192, 168, 2, 1)),
        dns_servers: Default::default(),
    });

    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stacks
    let (ap_stack, ap_runner) = embassy_net::new(
        wifi_ap_device,
        ap_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    let (sta_stack, sta_runner) = embassy_net::new(
        wifi_sta_device,
        sta_config,
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    let client_config = Configuration::Mixed(
        ClientConfiguration {
            ssid: SSID.try_into().unwrap(),
            password: PASSWORD.try_into().unwrap(),
            ..Default::default()
        },
        AccessPointConfiguration {
            ssid: "esp-wifi".try_into().unwrap(),
            ..Default::default()
        },
    );
    controller.set_configuration(&client_config).unwrap();

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task_ap(ap_runner)).ok();
    spawner.spawn(net_task_sta(sta_runner)).ok();

    let sta_address = loop {
        if let Some(config) = sta_stack.config_v4() {
            let address = config.address.address();
            info!("Got IP: {}", address);
            break address;
        }
        info!("Waiting for IP...");
        Timer::after(Duration::from_millis(500)).await;
    };
    loop {
        if ap_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Connect to the ap '{SSID}' and connect to {sta_address} port 8080");

    let mut ap_server_socket = TcpSocket::new(ap_stack, ap_server_rx_buffer, ap_server_tx_buffer);
    ap_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let mut sta_server_socket =
        TcpSocket::new(sta_stack, sta_server_rx_buffer, sta_server_tx_buffer);
    sta_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
    (ap_server_socket, sta_server_socket)
}

// from example
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());

    info!("Starting wifi");
    controller.start_async().await.unwrap();
    info!("Wifi started!");

    loop {
        match esp_wifi::wifi::ap_state() {
            WifiState::ApStarted => {
                info!("About to connect...");

                match controller.connect_async().await {
                    Ok(_) => {
                        // wait until we're no longer connected
                        controller.wait_for_event(WifiEvent::StaDisconnected).await;
                        info!("STA disconnected");
                    }
                    Err(e) => {
                        error!("Failed to connect to wifi: {e:?}");
                        Timer::after(Duration::from_millis(5000)).await
                    }
                }
            }
            _ => return,
        }
    }
}

// from example
#[embassy_executor::task(pool_size = 2)]
async fn net_task_ap(
    mut runner: Runner<'static, WifiDevice<'static, esp_wifi::wifi::WifiApDevice>>,
) {
    runner.run().await
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task_sta(
    mut runner: Runner<'static, WifiDevice<'static, esp_wifi::wifi::WifiStaDevice>>,
) {
    runner.run().await
}
