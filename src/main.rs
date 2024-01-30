#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

mod fmt;
mod lights;
#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use debounced_pin::{DebouncedInputPin, ActiveHigh, DebounceState, Debounce};

    use hal::gpio::Interrupt;
    use rp_pico::hal;

    use crate::lights::TrafficLight;

    type ButtonPin = DebouncedInputPin<
        hal::gpio::Pin<
            hal::gpio::bank0::Gpio14,
            hal::gpio::PullUpInput,
        >,
        ActiveHigh,
    >;

    #[shared]
    struct Shared {
        traffic_light: TrafficLight,
        button: ButtonPin,
        state: i32,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {

        let mut resets = c.device.RESETS;
        
        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        
        let button = pins.gpio14.into_pull_up_input();
        button.set_interrupt_enabled(Interrupt::EdgeLow, true);
        let button = DebouncedInputPin::new(button, ActiveHigh);
        
        let red_led = pins.gpio13.into_push_pull_output();
        let yellow_led = pins.gpio15.into_push_pull_output();
        let green_led = pins.gpio16.into_push_pull_output();
        let state = 0;

        let traffic_light = TrafficLight::new(
            red_led,
            yellow_led,
            green_led,
        );        
        
        rtic::pend(hal::pac::Interrupt::IO_IRQ_BANK0);
        (
            Shared {
                traffic_light,
                button,
                state,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(
        binds = IO_IRQ_BANK0,
        shared = [traffic_light, button, state]
    )]
    fn io_bank0(cx: io_bank0::Context) {
        let traffic_light = cx.shared.traffic_light;
        let button = cx.shared.button;
        let state = cx.shared.state;

        (
            traffic_light,
            button,
            state
        ).lock(|traffic_light, button,state| {
            match button.update().unwrap() {
                DebounceState::Debouncing => return,
                DebounceState::Reset => return,
                DebounceState::NotActive => return,
                DebounceState::Active => {
                    match state {
                        1 => {
                            traffic_light.yellow();
                        }
                        2 => {
                            traffic_light.green();
                        }
                        _ => {
                            traffic_light.red();
                            *state = 0;
                        }
                    }
                    *state += 1;
    
                    button.pin.clear_interrupt(Interrupt::EdgeLow);
                }
            }
        });
    }

    // Task with least priority that only runs when nothing else is running.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rtic::pend(hal::pac::Interrupt::IO_IRQ_BANK0);
        loop {
            cortex_m::asm::nop();
        }
    }

}
