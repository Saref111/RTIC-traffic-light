use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal;

type RedLedPin = hal::gpio::Pin<
    hal::gpio::bank0::Gpio13,
    hal::gpio::PushPullOutput,
>;

type YellowLedPin = hal::gpio::Pin<
    hal::gpio::bank0::Gpio15,
    hal::gpio::PushPullOutput,
>;
type GreenLedPin = hal::gpio::Pin<
    hal::gpio::bank0::Gpio16,
    hal::gpio::PushPullOutput,
>;

pub struct TrafficLight {
    red_led: RedLedPin,
    yellow_led: YellowLedPin,
    green_led: GreenLedPin,
    state: i32,
}

impl TrafficLight {
    pub fn new(
        red_led: RedLedPin,
        yellow_led: YellowLedPin,
        green_led: GreenLedPin,
    ) -> Self {
        TrafficLight {
            red_led,
            yellow_led,
            green_led,
            state: 0,
        }
    }

    pub fn red(&mut self) {
        self.red_led.set_high().unwrap();
        self.yellow_led.set_low().unwrap();
        self.green_led.set_low().unwrap();
    }

    pub fn yellow(&mut self) {
        self.red_led.set_low().unwrap();
        self.yellow_led.set_high().unwrap();
        self.green_led.set_low().unwrap();
    }

    pub fn green(&mut self) {
        self.red_led.set_low().unwrap();
        self.yellow_led.set_low().unwrap();
        self.green_led.set_high().unwrap();
    }

    pub fn increment(&mut self) {
        self.state += 1;
        if self.state > 2 {
            self.state = 0;
        }
    }

    pub fn update(&mut self) {
        match self.state {
            1 => {
                self.yellow();
            }
            2 => {
                self.green();
            }
            _ => {
                self.red();
            }
        }

        self.increment();
    }
}
