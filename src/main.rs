#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use hd44780_driver::{memory_map::MemoryMap1602, setup::DisplayOptionsI2C, HD44780};
use panic_probe as _;
use rp_pico::{
    self as bsp,
    hal::{fugit::RateExtU32, Timer, I2C},
    Gp18I2C1Sda, Gp19I2C1Scl,
};
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use embedded_hal::digital::InputPin;

#[entry]
fn main() -> ! {
	info!("Program start");
	let mut pac = pac::Peripherals::take().unwrap();
	let core = pac::CorePeripherals::take().unwrap();
	let mut watchdog = Watchdog::new(pac.WATCHDOG);
	let sio = Sio::new(pac.SIO);

	let external_xtal_freq_hz = 12_000_000u32;
	let clocks = init_clocks_and_plls(
		external_xtal_freq_hz,
		pac.XOSC,
		pac.CLOCKS,
		pac.PLL_SYS,
		pac.PLL_USB,
		&mut pac.RESETS,
		&mut watchdog,
	)
	.ok()
	.unwrap();

	let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

	let pins = bsp::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

	let sda: Gp18I2C1Sda = pins.gpio18.reconfigure();
	let scl: Gp19I2C1Scl = pins.gpio19.reconfigure();
	let mut ir_out = pins.gpio13.into_pull_down_input();
	let i2c = I2C::i2c1(pac.I2C1, sda, scl, 100.kHz(), &mut pac.RESETS, &clocks.system_clock);
	
	let mut options = DisplayOptionsI2C::new(MemoryMap1602::new()).with_i2c_bus(i2c, 0x27);
	let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

	let mut display = loop {
		match HD44780::new(options, &mut timer) {
			Err((options_back, error)) => {
				error!("Error creating LCD Driver: {}", error);
				options = options_back;
				delay.delay_ms(500);
			}
			Ok(display) => break display,
		}
	};

	loop {
		if ir_out.is_high().unwrap() {
            let start_time = timer.get_counter_low();

            while ir_out.is_high().unwrap() {}

            let pulse_duration = timer.get_counter_low() - start_time;

            if pulse_duration >= 9000 && pulse_duration <= 10000 {
                
                let mut data = 0u32;
                for _ in 0..32 {
                    while ir_out.is_low().unwrap() {}

                    let high_start = timer.get_counter_low();
                    while ir_out.is_high().unwrap() {}
                    let bit_duration = timer.get_counter_low() - high_start;

                    data <<= 1;
                    if bit_duration > 1200 {
                        data |= 1;
                    }
                }

				let mut str_data = [0u8; 8];
                for i in (0..8).rev() { // Converts to hexidecimal
                    let nibble = (data >> (i * 4)) & 0xF;
                    str_data[i] = if nibble < 10 {
                        b'0' + nibble as u8
                    } else {
                        b'A' + (nibble - 10) as u8
                    };
                }

				let str_data = core::str::from_utf8(&str_data).unwrap();

				display.clear(&mut timer).unwrap();
				display.reset(&mut timer).unwrap();
				
				display.set_cursor_xy((0, 0), &mut timer).unwrap();
				display.write_str(str_data, &mut timer).unwrap();
            }
        }
	}
}