//! VirtIO guest drivers.

#![feature(slice_ptr_len)]
#![cfg_attr(not(test), no_std)]
#![deny(unused_must_use, missing_docs)]
#![allow(clippy::identity_op)]
#![allow(dead_code)]

#[cfg(any(feature = "alloc", test))]
extern crate alloc;

mod blk;
mod console;
mod gpu;
mod hal;
#[cfg(feature = "alloc")]
mod input;
mod net;
mod queue;
mod transport;
mod volatile;
mod vsock;

pub use self::blk::{BlkResp, RespStatus, VirtIOBlk};
pub use self::console::VirtIOConsole;
pub use self::gpu::VirtIOGpu;
pub use self::hal::{Hal, PhysAddr, VirtAddr};
#[cfg(feature = "alloc")]
pub use self::input::{InputConfigSelect, InputEvent, VirtIOInput};
pub use self::net::VirtIONet;
use self::queue::VirtQueue;
pub use self::transport::mmio::{MmioError, MmioTransport, MmioVersion, VirtIOHeader};
pub use self::transport::pci;
pub use self::transport::{DeviceStatus, DeviceType, Transport};
pub use self::vsock::{VSockOp, VirtIOVsock, SEND_RECV_DATA_BUF_SIZE};
use core::mem::size_of;
use hal::*;

/// The page size in bytes supported by the library (4 KiB).
pub const PAGE_SIZE: usize = 0x1000;

/// The type returned by driver methods.
pub type Result<T = ()> = core::result::Result<T, Error>;

/// The error type of VirtIO drivers.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// The buffer is too small.
    BufferTooSmall,
    /// The device is not ready.
    NotReady,
    /// The queue is already in use.
    AlreadyUsed,
    /// Invalid parameter.
    InvalidParam,
    /// Failed to alloc DMA memory.
    DmaError,
    /// I/O Error
    IoError,
    /// The config space advertised by the device is smaller than the driver expected.
    ConfigSpaceTooSmall,
    /// The device doesn't have any config space, but the driver expects some.
    ConfigSpaceMissing,
}

/// Align `size` up to a page.
fn align_up(size: usize) -> usize {
    (size + PAGE_SIZE) & !(PAGE_SIZE - 1)
}

/// The number of pages required to store `size` bytes, rounded up to a whole number of pages.
fn pages(size: usize) -> usize {
    (size + PAGE_SIZE - 1) / PAGE_SIZE
}

/// Convert a struct into a byte buffer.
unsafe trait AsBuf: Sized {
    fn as_buf(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const _ as _, size_of::<Self>()) }
    }
    fn as_buf_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as _, size_of::<Self>()) }
    }
}
