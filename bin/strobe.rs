#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]


mod hd108;
use crate::hd108::HD108;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::channel::Receiver;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiBus;
use esp_backtrace as _;
use esp_hal::dma::DmaDescriptor;
use esp_hal::spi::master::prelude::_esp_hal_spi_master_dma_WithDmaSpi2;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    gpio::{Event, GpioPin, Input, Io, Pull},
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::println;
use heapless08::Vec;
use panic_halt as _;
use static_cell::StaticCell;

enum Message {
    ButtonPressed,
}

static SIGNAL_CHANNEL: StaticCell<Channel<NoopRawMutex, Message, 1>> = StaticCell::new();


#[embassy_executor::task]
async fn button_task(
    mut button_pin: Input<'static, GpioPin<10>>,
    sender: Sender<'static, NoopRawMutex, Message, 1>,
) {
    loop {
        // Wait for a button press
        button_pin.wait_for_falling_edge().await;
        sender.send(Message::ButtonPressed).await;
        Timer::after(Duration::from_millis(400)).await; // Debounce delay

    }
}


#[embassy_executor::task]
async fn led_task(
    mut hd108: HD108<impl SpiBus<u8> + 'static>,
    receiver: Receiver<'static, NoopRawMutex, Message, 1>,
) {
    let led_count = 97;

    // Predefined durations for the strobe effect in milliseconds
    let durations = [100, 200, 300, 400, 500];

    // Predefined colors (red, green, blue values)
    let colors = [
        (255, 0, 0),   // Red
        (0, 255, 0),   // Green
        (0, 0, 255),   // Blue
        (255, 255, 0), // Yellow
        (0, 255, 255), // Cyan
        (255, 0, 255), // Magenta
        (255, 255, 255), // White
    ];

    let mut iteration_count = 0;

    while iteration_count < 50 {
        let mut led_updates: heapless08::Vec<(usize, u8, u8, u8), 97> = heapless08::Vec::new();

        // Select color based on the current iteration count
        let color = colors[iteration_count % colors.len()];

        // Set all LEDs to the current color
        for i in 0..led_count {
            led_updates.push((i, color.0, color.1, color.2)).unwrap();
        }

        // Set the LEDs with the current color
        hd108.set_leds(&led_updates).await.unwrap();

        // Select duration based on the current iteration count
        let duration_ms = durations[iteration_count % durations.len()];
        Timer::after(Duration::from_millis(duration_ms)).await;

        // Turn off all LEDs briefly before the next strobe effect
        hd108.set_off().await.unwrap();
        Timer::after(Duration::from_millis(100)).await; // Brief pause between strobes

        iteration_count += 1;
    }

    println!("Strobe animation complete...");

    // Set all leds off
    hd108.set_off().await.unwrap();

    loop {
       
    }
}


#[main]
async fn main(spawner: Spawner) {
    println!("Starting program!...");

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let sclk = io.pins.gpio6;
    let miso = io.pins.gpio8;
    let mosi = io.pins.gpio7;
    let cs = io.pins.gpio9;


    let dma = Dma::new(peripherals.DMA);

    let dma_channel = dma.channel0;

    static TX_DESC: StaticCell<[DmaDescriptor; 8]> = StaticCell::new();
    let tx_descriptors = TX_DESC.init([DmaDescriptor::EMPTY; 8]);

    static RX_DESC: StaticCell<[DmaDescriptor; 8]> = StaticCell::new();
    let rx_descriptors = RX_DESC.init([DmaDescriptor::EMPTY; 8]);

    let spi = Spi::new(peripherals.SPI2, 20.MHz(), SpiMode::Mode0, &clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
        .with_dma(dma_channel.configure_for_async(
            false,
            tx_descriptors,
            rx_descriptors,
            DmaPriority::Priority0,
        ));

    let hd108 = HD108::new(spi);

    // Initialize the button pin as input with interrupt and pull-up resistor
    let mut button_pin = Input::new(io.pins.gpio10, Pull::Up);

    // Enable interrupts for the button pin
    button_pin.listen(Event::FallingEdge);

    let signal_channel = SIGNAL_CHANNEL.init(Channel::new());

    // Spawn the button task with ownership of the button pin and the sender
    spawner
        .spawn(button_task(button_pin, signal_channel.sender()))
        .unwrap();

    // Spawn the led task with the receiver
    spawner
        .spawn(led_task(hd108, signal_channel.receiver()))
        .unwrap();
}
