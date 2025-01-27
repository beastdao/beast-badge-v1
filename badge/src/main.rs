#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use embedded_hal_0_2::digital::v2::OutputPin;
use pimoroni_tufty2040 as tufty;
// The macro for our start-up function
use tufty::entry;
use tufty::DummyPin;
// GPIO traits
use embedded_hal::digital::PinState;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use tufty::hal;

use hal::clocks::ClockSource;
use hal::gpio::{FunctionPio0, PullNone};
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;
use hal::Clock;
use hal::Timer;

use button::Button;
use embedded_graphics::{
    draw_target::DrawTarget,
    image::Image,
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::{Rgb565, RgbColor},
    prelude::*,
    text::{Alignment, Text},
    Drawable,
};
use st7789::{Orientation, ST7789};
use tinybmp::Bmp;

enum Category {
    Badge,
    BadgeEvent,
    LinkTree,
}

struct Menu {
    category: Category,
    page: u8,
}

impl Menu {
    fn new() -> Self {
        Self {
            category: Category::Badge,
            page: 0,
        }
    }
    // overlapping 0->1->2->0->1..
    fn next_page(&mut self) {
        self.page = (self.page + 1) % 3;
    }
    // overlapping 0->2->1->0->2..
    fn previous_page(&mut self) {
        self.page = (self.page + 2) % 3;
    }
}

fn show_image<D>(display: &mut D, img: &[u8], position: Point)
where
    D: DrawTarget<Color = Rgb565>,
    <D as DrawTarget>::Error: core::fmt::Debug,
{
    let bmp: Bmp<Rgb565> = Bmp::from_slice(img).unwrap();
    Image::new(&bmp, position).draw(display).unwrap();
}

fn show_text<D>(display: &mut D, text: &str, position: Point)
where
    D: DrawTarget<Color = Rgb565>,
    <D as DrawTarget>::Error: core::fmt::Debug,
{
    Text::with_alignment(
        text,
        position,
        MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE),
        Alignment::Left,
    )
    .draw(display)
    .unwrap();
}

fn update_screen<D>(display: &mut D, current_state: &Menu)
where
    D: DrawTarget<Color = Rgb565>,
    <D as DrawTarget>::Error: core::fmt::Debug,
{
    let img1 = include_bytes!("../../assets/big.bmp");
    let img2 = include_bytes!("../../assets/qrl.bmp");
    let img3 = include_bytes!("../../assets/yb.bmp");
    let img4 = include_bytes!("../../assets/qra.bmp");
    let img5 = include_bytes!("../../assets/yethz.bmp");
    let img6 = include_bytes!("../../assets/yz.bmp");
    match current_state.category {
        Category::Badge => match current_state.page {
            0 => {
                show_image(display, img3, Point::new(0, 0));
            }
            1 => {
                display.clear(Rgb565::BLACK).unwrap();
                show_image(display, img1, Point::new(0, 0));
                show_text(display, "YULIA @ BEAST", Point::new(100, 200));
            }
            _ => {
                display.clear(Rgb565::WHITE).unwrap();
                show_image(display, img4, Point::new(40, 0));
            }
        },
        Category::BadgeEvent => match current_state.page {
            0 => {
                show_image(display, img5, Point::new(0, 0));
            }
            1 => {
                display.clear(Rgb565::new(12, 11, 19)).unwrap();
                show_image(display, img6, Point::new(0, 0));
                show_text(display, "YULIA @ ETHZURICH", Point::new(85, 200));
            }
            _ => {
                display.clear(Rgb565::WHITE).unwrap();
                show_image(display, img4, Point::new(40, 0));
            }
        },
        Category::LinkTree => match current_state.page {
            0 => {
                display.clear(Rgb565::WHITE).unwrap();
                show_image(display, img2, Point::new(40, 0));
            }
            _ => {}
        },
    }
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        tufty::XOSC_CRYSTAL_FREQ, //can replace
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = tufty::Pins::new(
        //bsp
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the timer peripheral for our blinky delay
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.get_freq().to_Hz());

    pins.lcd_backlight
        .into_push_pull_output_in_state(PinState::High);
    pins.lcd_rd.into_push_pull_output_in_state(PinState::High);

    let display_data = {
        use hal::dma::DMAExt;
        use hal::pio::PIOExt;

        let dma = pac.DMA.split(&mut pac.RESETS);
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

        let wr = pins.lcd_wr.reconfigure::<FunctionPio0, PullNone>();
        let d0 = pins.lcd_db0.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db1.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db2.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db3.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db4.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db5.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db6.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db7.reconfigure::<FunctionPio0, PullNone>();

        tufty::PioDataLines::new(
            &mut pio,
            clocks.system_clock.freq(),
            wr.id(),
            d0.id(),
            sm0,
            dma.ch0,
        )
    };

    let display_interface = tufty::ParallelDisplayInterface::new(
        pins.lcd_cs.into_push_pull_output_in_state(PinState::High),
        pins.lcd_dc.into_push_pull_output_in_state(PinState::High),
        display_data,
    );

    // batetery level
    let full_battery: f32 = 3.7;
    let empty_battery: f32 = 2.5;
    let mut last_battery_check = timer.get_counter();
    let battery_check_interval_ms = 10000; // 10 sec

    let mut led_pin = pins.led.into_push_pull_output();

    //battery sensor

    use embedded_hal_0_2::adc::OneShot;

    let vref_en = pins
        .sensor_power
        .into_push_pull_output_in_state(PinState::Low);

    let mut adc = rp2040_hal::Adc::new(pac.ADC, &mut pac.RESETS);
    adc.take_temp_sensor().unwrap();
    let mut adc_pin_vbat =
        pimoroni_tufty2040::hal::adc::AdcPin::new(pins.vbat_sense.into_floating_input()).unwrap();
    let mut adc_pin_vref =
        pimoroni_tufty2040::hal::adc::AdcPin::new(pins.vref_1v24.into_floating_input()).unwrap();

    vref_en.into_push_pull_output_in_state(PinState::High);

    // 5 buttons

    let button_a_pin = pins.sw_a.into_pull_down_input().into_dyn_pin();
    let button_b_pin = pins.sw_b.into_pull_down_input().into_dyn_pin();
    let button_c_pin = pins.sw_c.into_pull_down_input().into_dyn_pin();
    let button_up_pin = pins.sw_up.into_pull_down_input().into_dyn_pin();
    let button_down_pin = pins.sw_down.into_pull_down_input().into_dyn_pin();
    let mut button_a = Button::new(button_a_pin, timer);
    let mut button_b = Button::new(button_b_pin, timer);
    let mut button_c = Button::new(button_c_pin, timer);
    let mut button_up = Button::new(button_up_pin, timer);
    let mut button_down = Button::new(button_down_pin, timer);

    let intro_img = include_bytes!("../../assets/blw3.bmp");

    let mut display = ST7789::new(display_interface, DummyPin, 240, 320);
    display.init(&mut delay).unwrap();
    display
        .set_orientation(Orientation::LandscapeSwapped)
        .unwrap();

    display.clear(Rgb565::WHITE).unwrap();
    show_image(&mut display, intro_img, Point::new(0, 40));
    timer.delay_ms(3000);
    display.clear(Rgb565::BLACK).unwrap();
    let mut current_state = Menu::new();
    update_screen(&mut display, &current_state);

    loop {
        // battery level
        let now = timer.get_counter();
        if (now - last_battery_check).to_millis() >= battery_check_interval_ms {
            last_battery_check = now;
            let vref_val: u16 = adc.read(&mut adc_pin_vref).unwrap();
            let vbat_val: u16 = adc.read(&mut adc_pin_vbat).unwrap();
            let vdd: f32 = 1.24 * (65535.0 / vref_val as f32);
            let vbat: f32 = vbat_val as f32 * 3.0 * vdd / 65535.0;
            let mut percentage: f32 =
                100.0 * ((vbat - empty_battery) / (full_battery - empty_battery));
            percentage = percentage.clamp(0.0, 100.0);
            if percentage < 20.0 {
                led_pin.set_high().unwrap();
                timer.delay_ms(100);
                led_pin.set_low().unwrap();
            }
        }

        if button_a.is_button_pressed() {
            current_state.category = Category::Badge;
            current_state.page = 0;
            update_screen(&mut display, &current_state);
        }
        if button_b.is_button_pressed() {
            current_state.category = Category::LinkTree;
            current_state.page = 0;
            update_screen(&mut display, &current_state);
        }
        if button_c.is_button_pressed() {
            current_state.category = Category::BadgeEvent;
            current_state.page = 0;
            update_screen(&mut display, &current_state);
        }

        if button_up.is_button_pressed() {
            current_state.next_page();
            update_screen(&mut display, &current_state);
        }
        if button_down.is_button_pressed() {
            current_state.previous_page();
            update_screen(&mut display, &current_state);
        }
        //small delay
        timer.delay_ms(50);
    }
}
