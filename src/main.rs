#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

mod fmt;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    use debounced_pin::{DebouncedInputPin, ActiveHigh, DebounceState, Debounce};

    use hal::gpio::Interrupt;
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

    type ButtonPin = DebouncedInputPin<
        hal::gpio::Pin<
            hal::gpio::bank0::Gpio14,
            hal::gpio::PullUpInput,
        >,
        ActiveHigh,
    >;

    #[shared]
    struct Shared {
        red_led: RedLedPin,
        yellow_led: YellowLedPin,
        green_led: GreenLedPin,
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
        
        
        rtic::pend(hal::pac::Interrupt::IO_IRQ_BANK0);
        (
            Shared {
                button,
                red_led,
                yellow_led,
                green_led,
                state,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(
        binds = IO_IRQ_BANK0,
        shared = [red_led, yellow_led, green_led, button, state]
    )]
    fn io_bank0(cx: io_bank0::Context) {
        let red_led = cx.shared.red_led;
        let yellow_led = cx.shared.yellow_led;
        let green_led = cx.shared.green_led;
        let button = cx.shared.button;
        let state = cx.shared.state;


        (
            red_led,
            yellow_led,
            green_led,
            button,
            state
        ).lock(|
            red_led,
            yellow_led,
            green_led,
            button,
            state
            | {
            match button.update().unwrap() {
                DebounceState::Debouncing => return,
                DebounceState::Reset => return,
                DebounceState::NotActive => return,
                DebounceState::Active => {
                    match state {
                        1 => {
                            red_led.set_low().unwrap();
                            yellow_led.set_high().unwrap();
                            green_led.set_low().unwrap();
                        }
                        2 => {
                            red_led.set_low().unwrap();
                            yellow_led.set_low().unwrap();
                            green_led.set_high().unwrap();
                        }
                        _ => {
                            red_led.set_high().unwrap();
                            yellow_led.set_low().unwrap();
                            green_led.set_low().unwrap();
                            *state = 0;
                        }
                    }
                    *state += 1;
    
                    button.pin.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
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
