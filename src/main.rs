#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use libm::round;

use cortex_m_rt::entry;
use microbit::{
    hal::prelude::*,
    hal::{
        clocks::Clocks,
        gpio,
        prelude::OutputPin,
        pwm,
        rtc::{Rtc, RtcInterrupt},
        time::Hertz,
        Timer,
    },
    pac::{self, interrupt},
    Board,
};

const BASE_INTERVAL: u32 = 128;
static RTC: Mutex<RefCell<Option<Rtc<pac::RTC0>>>> = Mutex::new(RefCell::new(None));
static SPEAKER: Mutex<RefCell<Option<pwm::Pwm<pac::PWM0>>>> = Mutex::new(RefCell::new(None));
static INTERVAL: Mutex<RefCell<f64>> = Mutex::new(RefCell::new(BASE_INTERVAL as f64));

fn to_bpm(interval: f64) -> f64 {
    round(BASE_INTERVAL as f64 / interval * 60.0)
}

fn to_interval(bpm: f64) -> f64 {
    BASE_INTERVAL as f64 / bpm * 60.0
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Start");

    if let Some(mut board) = Board::take() {
        let button_a = board.buttons.button_a.into_pullup_input();
        let button_b = board.buttons.button_b.into_pullup_input();
        let _clocks = Clocks::new(board.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_synth()
            .start_lfclk();

        let mut timer = Timer::new(board.TIMER0);

        // Interrupt every 1/128 s  (32768 / 128 Hz) - 1 = 255
        let prescaler = 255; // 128 Hz
        let mut rtc = Rtc::new(board.RTC0, prescaler).unwrap();
        rtc.enable_counter();
        rtc.enable_interrupt(RtcInterrupt::Tick, Some(&mut board.NVIC));
        rtc.enable_event(RtcInterrupt::Tick);

        // For jack output use board.pins.p0_02 and large pin 0 on the board
        // let mut speaker_pin = board.pins.p0_02.into_push_pull_output(gpio::Level::Low);
        let mut speaker_pin = board.speaker_pin.into_push_pull_output(gpio::Level::Low);

        // Use the PWM peripheral to generate a waveform for the speaker
        let speaker = pwm::Pwm::new(board.PWM0);
        speaker
            .set_output_pin(pwm::Channel::C0, speaker_pin.degrade())
            .set_prescaler(pwm::Prescaler::Div16)
            .set_period(Hertz(440u32))
            .set_counter_mode(pwm::CounterMode::UpAndDown)
            .set_max_duty(32767)
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
                cortex_m::interrupt::free(move |cs| {
                    let mut interval = INTERVAL.borrow(cs).borrow_mut();
                    let bpm = to_bpm(*interval) - 1.0;
                    if bpm > 0.0 {
                        *interval = to_interval(bpm);
                    }
                    rprintln!("----BPM: {}, interval: {}", bpm, interval);
                });
                timer.delay_ms(100_u32);
            };

            if let Ok(true) = button_b.is_low() {
                cortex_m::interrupt::free(move |cs| {
                    let mut interval = INTERVAL.borrow(cs).borrow_mut();
                    if *interval > 0.0 {
                        let bpm = to_bpm(*interval) + 1.0;
                        *interval = to_interval(bpm);
                        rprintln!("----BPM: {}, interval: {}", bpm, interval);
                    };
                });
                timer.delay_ms(100_u32);
            };
        }
    }

    loop {
        continue;
    }
}

// RTC interrupt, executed for each RTC tick
#[interrupt]
fn RTC0() {
    static mut SLEEP_COUNTER: u32 = 0;
    static BEEP_DURATION: f64 = 8.0;

    /* Enter critical section */
    cortex_m::interrupt::free(|cs| {
        /* Borrow devices */
        if let (Some(speaker), Some(rtc)) = (
            SPEAKER.borrow(cs).borrow().as_ref(),
            RTC.borrow(cs).borrow().as_ref(),
        ) {
            let interval = INTERVAL.borrow(cs).borrow();

            if *SLEEP_COUNTER as f64 >= *interval {
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
