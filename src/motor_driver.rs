use embedded_hal::PwmPin;
use rp_pico as bsp;

use bsp::hal::{
    gpio::{
        bank0::{Gpio2, Gpio3, Gpio6, Gpio7},
        Disabled, Pin, PullDown,
    },
    pwm::{Channel, ChannelId, FreeRunning, Pwm1, Pwm3, Slice, SliceId, SliceMode, A, B},
};

pub enum Direction {
    Forward,
    Backward,
}

pub struct Motor<'a, S: SliceId, M: SliceMode, C1: ChannelId, C2: ChannelId> {
    mf_pwm: &'a mut Channel<S, M, C1>,
    mb_pwm: &'a mut Channel<S, M, C2>,
}

impl<'a, S: SliceId> Motor<'a, S, FreeRunning, A, B> {
    pub fn set(&mut self, direction: Direction, speed: f32) {
        let speed = (speed * 655.35) as u16;
        match direction {
            Direction::Forward => {
                self.mf_pwm.set_duty(speed);
                self.mb_pwm.set_duty(0);
            }
            Direction::Backward => {
                self.mf_pwm.set_duty(0);
                self.mb_pwm.set_duty(speed);
            }
        }
    }

    pub fn stop(&mut self) {
        self.mf_pwm.set_duty(0);
        self.mb_pwm.set_duty(0);
    }
}

pub fn init_motor_1(
    pwm_1: &mut Slice<Pwm1, FreeRunning>,
    gpio_2: Pin<Gpio2, Disabled<PullDown>>,
    gpio_3: Pin<Gpio3, Disabled<PullDown>>,
) -> Motor<Pwm1, FreeRunning, A, B> {
    pwm_1.set_ph_correct();
    pwm_1.enable();

    let mf_pwm = &mut pwm_1.channel_a;
    mf_pwm.output_to(gpio_2);

    let mb_pwm = &mut pwm_1.channel_b;
    mb_pwm.output_to(gpio_3);

    Motor { mf_pwm, mb_pwm }
}

pub fn init_motor_2(
    pwm_3: &mut Slice<Pwm3, FreeRunning>,
    gpio_6: Pin<Gpio6, Disabled<PullDown>>,
    gpio_7: Pin<Gpio7, Disabled<PullDown>>,
) -> Motor<Pwm3, FreeRunning, A, B> {
    pwm_3.set_ph_correct();
    pwm_3.enable();

    let mf_pwm = &mut pwm_3.channel_a;
    mf_pwm.output_to(gpio_6);

    let mb_pwm = &mut pwm_3.channel_b;
    mb_pwm.output_to(gpio_7);

    Motor { mf_pwm, mb_pwm }
}
