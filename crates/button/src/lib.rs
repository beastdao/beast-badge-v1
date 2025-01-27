#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::InputPin;

pub struct Button<P: InputPin, D: DelayNs> {
    pub button: P,
    pub delay: D,
    pub is_pressed: bool,
}

impl<P: InputPin, D: DelayNs> Button<P, D> {
    pub fn new(button: P, delay: D) -> Self {
        Self {
            button,
            delay,
            is_pressed: false,
        }
    }

    pub fn is_button_pressed(&mut self) -> bool {
        if self.button.is_high().unwrap_or(false) && !self.is_pressed {
            self.delay.delay_ms(200);
            self.is_pressed = true;
            return true;
        }
        if self.button.is_low().unwrap_or(true) {
            self.is_pressed = false;
        }
        false
    }
}
