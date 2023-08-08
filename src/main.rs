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
        time::Hertz,
    },
    pac::{self, interrupt},
    Board,
};

static RTC: Mutex<RefCell<Option<Rtc<pac::RTC0>>>> = Mutex::new(RefCell::new(None));
static SPEAKER: Mutex<RefCell<Option<pwm::Pwm<pac::PWM0>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Start");

    if let Some(mut board) = Board::take() {
        // let buttons = board.buttons;
        // let button_a = board.buttons.button_a.into_pullup_input();

        cortex_m::interrupt::free(move |cs| {
            // NB: The LF CLK pin is used by the speaker
            let _clocks = Clocks::new(board.CLOCK)
                .enable_ext_hfosc()
                .set_lfclk_src_synth()
                .start_lfclk();


            // Interrupt every 1/8s  (32768 / 8 Hz) - 1 = 4095
            let prescaler = 1023; // 32 Hz
            let mut rtc = Rtc::new(board.RTC0, prescaler).unwrap();
            rtc.enable_counter();
            rtc.enable_interrupt(RtcInterrupt::Tick, Some(&mut board.NVIC));
            rtc.enable_event(RtcInterrupt::Tick);

            *RTC.borrow(cs).borrow_mut() = Some(rtc);

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

            // Configure 50% duty cycle
            // let max_duty = speaker.max_duty();
            // speaker.set_duty_on_common(max_duty / 2);

            *SPEAKER.borrow(cs).borrow_mut() = Some(speaker);

            // Configure RTC interrupt
            unsafe {
                pac::NVIC::unmask(pac::Interrupt::RTC0);
            }
            pac::NVIC::unpend(pac::Interrupt::RTC0);
        });

        // loop {
        //     if let Ok(true) = button_a.is_high() {
        //         rprintln!("Button A high");
        //     };
        // }
    }

    loop {
        continue;
    }

}

// const STOP_FREQUENCY: u32 = 440;



// RTC interrupt, exectued for each RTC tick
#[interrupt]
fn RTC0() {
    // static mut FREQUENCY: u32 = 1;
    static mut COUNTER: u32 = 0;
    static mut SLEEP_COUNTER: u32 = 8;

    /* Enter critical section */
    cortex_m::interrupt::free(|cs| {
        /* Borrow devices */
        if let (Some(speaker), Some(rtc)) = (
            SPEAKER.borrow(cs).borrow().as_ref(),
            RTC.borrow(cs).borrow().as_ref(),
        ) {


            if *SLEEP_COUNTER >= 32 {
                speaker.set_period(Hertz(440));
                let max_duty = speaker.max_duty();
                speaker.set_duty_on_common(max_duty / 4);

                *SLEEP_COUNTER = 0;
            } else {
                speaker.disable();
            }



            // if *FREQUENCY < STOP_FREQUENCY {
            //     // Configure the new frequency, must not be zero.
            //     // Will change the max_duty
            //     speaker.set_period(Hertz(*FREQUENCY));
            // } else {
            //     // Continue at frequency
            //     speaker.set_period(Hertz(STOP_FREQUENCY));
            // }
            // // Restart the PWM at 50% duty cycle
            // if *FREQUENCY >= STOP_FREQUENCY + 250 {
            //     defmt::info!("Fin");
            //     // Stop speaker and RTC
            //     speaker.disable();
            //     rtc.disable_counter();
            // };

            if *COUNTER > 1000 {
                speaker.disable();
                rtc.disable_counter();
            }
            // Clear the RTC interrupt
            rtc.reset_event(RtcInterrupt::Tick);
        }
    });
    // Increase the frequency
    // *FREQUENCY += 1;
    *COUNTER += 1;
    *SLEEP_COUNTER += 1;
}

