#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

mod fmt;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use defmt::info;
    use embedded_hal::digital::v2::OutputPin;
    use embedded_time::duration::Extensions;
    use debounced_pin::{DebouncedInputPin, ActiveHigh, DebounceState, Debounce};

    use hal::gpio::Interrupt;
    use rp_pico::hal;
    use rp_pico::XOSC_CRYSTAL_FREQ;

    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};
    // USB Communications Class Device support
    use usbd_serial::SerialPort;

    // Blinck time 5 seconds
    const SCAN_TIME_US: u32 = 5000000; //  200000; // 5000000;  // 1000000; // 200000;

    // IMPORTANT: The USB-Serial with RTIC github project example that I'm following.
    //            I tried to use the Pico board examples of USB-Serial (without interrupts
    //            and with interrupts with success, but when using with RTIC I could not make
    //            it work when merged with the RTIC example.) So I asked some questions
    //            in the in Matrix chat and received links to examples of there github
    //            project where it was working, then a used and adapted some parts there
    //            in this project template.
    //            This were the kind folks that helped me in the Matrix chat, the 3 projects
    //            that they suggest me to study are good examples of programs made with RTIC
    //            and USB and should be studied.
    //
    // Paul Daniel Faria
    // https://github.com/Nashenas88/dactyl-manuform-kb2040-rs/blob/main/src/main.rs#L80
    //
    // see also:
    // korken89
    // https://github.com/korken89/pico-probe/tree/master/src
    //
    // see also:
    // Mathias
    // https://github.com/mgottschlag/rp2040-usb-sound-card/blob/b8078b57361c1b08755e5ab5f9992c56457ec18b/src/main.rs#L188
    //
    //
    // Global Static variable, has to be written inside unsafe blocks.
    // A reference can be obtained with as_ref() method.

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
        // timer: hal::Timer,
        // alarm: hal::timer::Alarm0,

        red_led: RedLedPin,
        yellow_led: YellowLedPin,
        green_led: GreenLedPin,
        button: ButtonPin,
        state: i32,
    }

    #[local]
    struct Local {}

    #[init(local = [usb_bus: Option<usb_device::bus::UsbBusAllocator<hal::usb::UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        //*******
        // Initialization of the system clock.

        let mut resets = c.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(c.device.WATCHDOG);

        // Configure the clocks - The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        //*******
        // Initialization of the USB and Serial and USB Device ID.

        // USB
        //
        // Set up the USB driver
        // The bus that is used to manage the device and class below.
        let usb_bus: &'static _ =
            c.local
                .usb_bus
                .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
                    c.device.USBCTRL_REGS,
                    c.device.USBCTRL_DPRAM,
                    clocks.usb_clock,
                    true,
                    &mut resets,
                )));

        // Set up the USB Communications Class Device driver.
        let _serial = SerialPort::new(usb_bus);

        // Create a USB device with a fake VID and PID
        let _usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        //*******
        // Initialization of the LED GPIO and the timer.

        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        // let mut alarm = timer.alarm_0().unwrap();
        // let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        // alarm.enable_interrupt(&mut timer);
        
        let button = pins.gpio14.into_pull_up_input();
        button.set_interrupt_enabled(Interrupt::EdgeLow, true);
        let button = DebouncedInputPin::new(button, ActiveHigh);
        
        let red_led = pins.gpio13.into_push_pull_output();
        let yellow_led = pins.gpio15.into_push_pull_output();
        let green_led = pins.gpio16.into_push_pull_output();
        let state = 0;
        
        
        rtic::pend(hal::pac::Interrupt::IO_IRQ_BANK0);
        info!("init");
        //********
        // Return the Shared variables struct, the Local variables struct and the XPTO Monitonics
        //    (Note: Read again the RTIC book in the section of Monotonics timers)
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

    /// Task that blinks the rp-pico onboard LED and that send a message "LED ON!" and "LED OFF!" do USB-Serial.
    #[task(
        binds = IO_IRQ_BANK0,
        priority = 1,
        shared = [red_led, yellow_led, green_led, button, state],
        local = [],
    )]
    fn io_bank0(cx: io_bank0::Context) {
        info!("io_bank0");
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
                        0 => {
                            red_led.set_high().unwrap();
                            yellow_led.set_low().unwrap();
                            green_led.set_low().unwrap();
                        }
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
                        _ => {}
                    }
                    *state += 1;
    
                    if *state > 2 {
                        *state = 0;
                    }
    
                    button.pin.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
                }
            }
        });
    }

    // Task with least priority that only runs when nothing else is running.
    #[idle(local = [x: u32 = 0])]
    fn idle(_cx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        // let _x: &'static mut u32 = cx.local.x;
        info!("idle");
        //hprintln!("idle").unwrap();
        rtic::pend(hal::pac::Interrupt::IO_IRQ_BANK0);
        loop {
            cortex_m::asm::nop();
        }
    }

    /* New Tasks */

}
