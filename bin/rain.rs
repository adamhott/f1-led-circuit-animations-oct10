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
    let led_count = 97;          // Total number of LEDs
    let group_size = 5;          // Each group will contain 5 LEDs
    let num_groups = led_count / group_size; // Number of groups
    let max_drops_per_group = 2; // Maximum number of raindrops per group

    // Array to hold the active raindrops for each group
    let mut raindrops: Vec<Vec<Option<(usize, u8, u8, u8)>, 2>, 20> = Vec::new();

    // Initialize each group with empty raindrop slots
    for _ in 0..num_groups {
        let mut group_drops: Vec<Option<(usize, u8, u8, u8)>, 2> = Vec::new();
        for _ in 0..max_drops_per_group {
            group_drops.push(None).unwrap();
        }
        raindrops.push(group_drops).unwrap();
    }

    let mut iteration_count = 0;

    while iteration_count < 100 {
        let mut led_updates: heapless08::Vec<(usize, u8, u8, u8), 97> = heapless08::Vec::new();

        // Set all LEDs to off (background)
        for i in 0..led_count {
            led_updates.push((i, 0, 0, 0)).unwrap();
        }

        // Update and move existing raindrops in each group
        for group_idx in 0..num_groups {
            let group_start = group_idx * group_size;  // Start index of the group
            let group_end = group_start + group_size;  // End index of the group

            for drop in &mut raindrops[group_idx] {
                if let Some((pos, r, g, b)) = drop {
                    // Render the current raindrop within the group
                    let led_pos = group_start + *pos;
                    led_updates[led_pos] = (led_pos, *r, *g, *b);

                    // Move the raindrop down (within the group)
                    if *pos < group_size - 1 {
                        *pos += 1;
                        // Fade the raindrop as it falls
                        *r = r.saturating_sub(30);
                        *g = g.saturating_sub(30);
                        *b = b.saturating_sub(30);
                    } else {
                        // Remove the raindrop when it reaches the bottom of the group
                        *drop = None;
                    }
                }
            }

            // Randomly spawn new raindrops at the top of each group
            for drop in &mut raindrops[group_idx] {
                if drop.is_none() && iteration_count % 10 == 0 { // Control the spawn rate
                    // Create a new raindrop at position 0 within the group
                    *drop = Some((
                        0,             // Position at the top of the group
                        255, 255, 255, // Bright white raindrop
                    ));
                    break; // Limit one raindrop per group per frame
                }
            }
        }

        // Apply the updated LED states
        hd108.set_leds(&led_updates).await.unwrap();
        Timer::after(Duration::from_millis(100)).await;

        iteration_count += 1;
    }

    println!("Rain animation complete...");

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
