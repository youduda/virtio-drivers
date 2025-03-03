//! Log implementation using the UART.

use crate::pl011::Uart;
use core::fmt::Write;
use log::{LevelFilter, Log, Metadata, Record, SetLoggerError};
use spin::mutex::SpinMutex;

/// Base memory-mapped address of the primary PL011 UART device.
pub const BASE_ADDRESS: usize = 0x900_0000;

static LOGGER: Logger = Logger {
    uart: SpinMutex::new(None),
};

struct Logger {
    uart: SpinMutex<Option<Uart>>,
}

/// Initialises UART logger.
pub fn init(max_level: LevelFilter) -> Result<(), SetLoggerError> {
    // Safe because BASE_ADDRESS is the base of the MMIO region for a UART and is mapped as device
    // memory.
    let uart = unsafe { Uart::new(BASE_ADDRESS) };
    LOGGER.uart.lock().replace(uart);

    log::set_logger(&LOGGER)?;
    log::set_max_level(max_level);
    Ok(())
}

impl Log for Logger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        writeln!(
            LOGGER.uart.lock().as_mut().unwrap(),
            "[{}] {}",
            record.level(),
            record.args()
        )
        .unwrap();
    }

    fn flush(&self) {}
}
