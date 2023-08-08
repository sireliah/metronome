#![no_main]
#![no_std]

// use defmt_rtt as _;
// use panic_halt as _;

use rtt_target::{rtt_init_print, rprintln};
use panic_rtt_target as _;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use cortex_m_rt::entry;
use microbit::{
    hal::prelude::*,
    hal::{
        clocks::Clocks,
        gpio,
        prelude::OutputPin,
        pwm,
        rtc::{Rtc, RtcInterrupt},
        time::Hertz, Timer,
    },
    pac::{self, interrupt},
    Board,
};

static RTC: Mutex<RefCell<Option<Rtc<pac::RTC0>>>> = Mutex::new(RefCell::new(None));
static SPEAKER: Mutex<RefCell<Option<pwm::Pwm<pac::PWM0>>>> = Mutex::new(RefCell::new(None));
static DURATION: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(32));

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Start");

    if let Some(mut board) = Board::take() {
        let button_a = board.buttons.button_a.into_pullup_input();
        let button_b = board.buttons.button_b.into_pullup_input();

        // NB: The LF CLK pin is used by the speaker
        let _clocks = Clocks::new(board.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_synth()
            .start_lfclk();

        let mut timer = Timer::new(board.TIMER0);

        // Interrupt every 1/32s  (32768 / 32 Hz) - 1 = 1023
        let prescaler = 1023; // 32 Hz
        let mut rtc = Rtc::new(board.RTC0, prescaler).unwrap();
        rtc.enable_counter();
        rtc.enable_interrupt(RtcInterrupt::Tick, Some(&mut board.NVIC));
        rtc.enable_event(RtcInterrupt::Tick);

        // To for speakers use board.pins.p0_02 and large pin 0 on the board

        let mut speaker_pin = board.speaker_pin.into_push_pull_output(gpio::Level::High);
        let _ = speaker_pin.set_low();

        // Use the PWM peripheral to generate a waveform for the speaker
        let speaker = pwm::Pwm::new(board.PWM0);
        speaker
            // output the waveform on the speaker pin
            .set_output_pin(pwm::Channel::C0, speaker_pin.degrade())
            // Use prescale by 16 to achive darker sounds
            .set_prescaler(pwm::Prescaler::Div16)
            // Initial frequency
            .set_period(Hertz(440u32))
            // Configure for up and down counter mode
            .set_counter_mode(pwm::CounterMode::UpAndDown)
            // Set maximum duty cycle
            .set_max_duty(32767)
            // enable PWM
            .enable();

        speaker
            .set_seq_refresh(pwm::Seq::Seq0, 0)
            .set_seq_end_delay(pwm::Seq::Seq0, 0);

        cortex_m::interrupt::free(move |cs| {

            *RTC.borrow(cs).borrow_mut() = Some(rtc);
            *SPEAKER.borrow(cs).borrow_mut() = Some(speaker);

            // Configure RTC interrupt
            unsafe {
                pac::NVIC::unmask(pac::Interrupt::RTC0);
            }
            pac::NVIC::unpend(pac::Interrupt::RTC0);
        });

        loop {
            if let Ok(true) = button_a.is_low() {
                rprintln!("Button A low");

                cortex_m::interrupt::free(move |cs| {
                    let mut duration = DURATION.borrow(cs).borrow_mut();
                    *duration += 1;
                    rprintln!("Duration: {}", *duration);
                });
                timer.delay_ms(200_u32);
            };

            if let Ok(true) = button_b.is_low() {
                rprintln!("Button B low");

                cortex_m::interrupt::free(move |cs| {
                    let mut duration = DURATION.borrow(cs).borrow_mut();
                    if *duration > 0 {
                        *duration -= 1;
                    };
                    rprintln!("Duration: {}", *duration);
                });
                timer.delay_ms(200_u32);
            };

        }
    }

    loop {
        continue;
    }

}


// RTC interrupt, exectued for each RTC tick
#[interrupt]
fn RTC0() {
    static mut SLEEP_COUNTER: u32 = 32;

    /* Enter critical section */
    cortex_m::interrupt::free(|cs| {
        /* Borrow devices */
        if let (Some(speaker), Some(rtc)) = (
            SPEAKER.borrow(cs).borrow().as_ref(),
            RTC.borrow(cs).borrow().as_ref(),
        ) {

            let duration = DURATION.borrow(cs).borrow();

            if *SLEEP_COUNTER >= *duration {
                speaker.set_period(Hertz(440));
                let max_duty = speaker.max_duty();
                speaker.set_duty_on_common(max_duty / 2);

                *SLEEP_COUNTER = 0;
            } else {
                speaker.disable();
            }

            // Clear the RTC interrupt
            rtc.reset_event(RtcInterrupt::Tick);
        }
    });
    *SLEEP_COUNTER += 1;
}

