use core::mem::{size_of, MaybeUninit};

use super::*;
use crate::transport::Transport;
use crate::volatile::{volread, volwrite, ReadOnly, Volatile};
use bitflags::*;
use core::hint::spin_loop;
use core::ptr::NonNull;
use log::*;

/// The Virtio Vsock device is a virtual socket.
///
/// It has enhanced rapidly and demonstrates clearly how support for new
/// features are added to an existing device.
/// Empty buffers are placed in one virtqueue for receiving packets, and
/// outgoing packets are enqueued into another for transmission in that order.
/// A third command queue is used to control advanced filtering features.
pub struct VirtIOVsock<H: Hal, T: Transport> {
    transport: T,
    guest_cid: u64,
    recv_queue: VirtQueue<H>,
    send_queue: VirtQueue<H>,
    event_queue: VirtQueue<H>,
    header_buf: NonNull<Header>,
}

impl<H: Hal, T: Transport> VirtIOVsock<H, T> {
    /// Create a new VirtIO-VSock driver.
    pub fn new(mut transport: T) -> Result<Self> {
        transport.begin_init(|features| {
            let features = Features::from_bits_truncate(features);
            info!("Device features {:?}", features);
            let supported_features = Features::VIRTIO_VSOCK_F_SEQPACKET;
            (features & supported_features).bits()
        });
        // read configuration space
        let config = transport.config_space::<Config>()?;
        let guest_cid;
        // Safe because config points to a valid MMIO region for the config space.
        unsafe {
            guest_cid = volread!(config, guest_cid);
        }
        debug!("Got guest_cid={:?}", guest_cid);
        // TODO: Assert guest_cid not reserved: virtio 5.10.4

        let queue_num = 2; // for simplicity
        let event_queue = VirtQueue::new(&mut transport, QUEUE_EVENT, queue_num)?;
        let recv_queue = VirtQueue::new(&mut transport, QUEUE_RECEIVE, queue_num)?;
        let send_queue = VirtQueue::new(&mut transport, QUEUE_TRANSMIT, queue_num)?;

        transport.finish_init();

        // Allocate a single page for the header buffer. This is used during send and recv.
        let dma_page = H::dma_alloc(1);
        let vaddr_dma_page = H::phys_to_virt(dma_page);
        let header_buf = NonNull::new(vaddr_dma_page as *mut Header)
            .expect("Allocation of header buffer failed");

        Ok(VirtIOVsock {
            transport,
            guest_cid,
            recv_queue,
            send_queue,
            event_queue,
            header_buf,
        })
    }

    /// Acknowledge interrupt.
    pub fn ack_interrupt(&mut self) -> bool {
        self.transport.ack_interrupt()
    }

    /// Get MAC address.
    pub fn guest_cid(&self) -> u64 {
        self.guest_cid
    }

    /// Whether can send packet.
    pub fn can_send(&self) -> bool {
        self.send_queue.available_desc() >= 2
    }

    /// Whether can receive packet.
    pub fn can_recv(&self) -> bool {
        self.recv_queue.can_pop()
    }

    /// Receive a packet.
    /// Returns a tuple (src_cid, src_port, dst_port, buf_len)
    pub fn recv(&mut self, buf: &mut [u8]) -> Result<(u64, u32, u32, usize)> {
        // Note: `header_buf` and `self.header_buf` point to the same memory address
        let header_buf = unsafe { self.header_buf.as_mut().as_buf_mut() };

        self.recv_queue.add(&[], &[header_buf, buf])?;
        self.transport.notify(QUEUE_RECEIVE);
        while !self.recv_queue.can_pop() {
            spin_loop();
        }

        let dst_cid = unsafe { volread!(self.header_buf, dst_cid) };
        assert_eq!(
            dst_cid, self.guest_cid,
            "TODO: What to to if received frame has different dst_cid"
        );
        // TODO: virtio 5.10.6.3: A VIRTIO_VSOCK_OP_RST reply MUST be sent if a packet is received with an unknown type

        let (_, len) = self.recv_queue.pop_used()?;
        Ok((
            unsafe { volread!(self.header_buf, src_cid) },
            unsafe { volread!(self.header_buf, src_port) },
            unsafe { volread!(self.header_buf, dst_port) },
            len as usize - size_of::<Header>(),
        ))
    }

    /// Send a packet.
    pub fn send(&mut self, src_port: u32, dst_cid: u64, dst_port: u32, buf: &[u8]) -> Result {
        // Note: `header_buf` and `self.header_buf` point to the same memory address
        let header_buf = unsafe { self.header_buf.as_mut().as_buf_mut() };
        unsafe {
            core::ptr::write_volatile(self.header_buf.as_ptr(), Header::default());
            volwrite!(self.header_buf, src_cid, self.guest_cid);
            volwrite!(self.header_buf, src_port, src_port);
            volwrite!(self.header_buf, dst_cid, dst_cid);
            volwrite!(self.header_buf, dst_port, dst_port);
        }

        self.send_queue.add(&[header_buf, buf], &[])?;
        self.transport.notify(QUEUE_TRANSMIT);
        while !self.send_queue.can_pop() {
            spin_loop();
        }
        self.send_queue.pop_used()?;
        Ok(())
    }

    pub fn recv_event() {
        // virtio 5.10.6.7
        todo!()
    }
}

// virtio 5.10.3 Feature bits
bitflags! {
    struct Features: u64 {
        /// stream socket type is supported
        const VIRTIO_VSOCK_F_STREAM = 1 << 0;
        /// seqpacket socket type is supported
        const VIRTIO_VSOCK_F_SEQPACKET = 1 << 1;

        // device independent
        const RING_INDIRECT_DESC = 1 << 28;
        const RING_EVENT_IDX = 1 << 29;
        const VERSION_1 = 1 << 32; // legacy
    }
}

bitflags! {
    struct InterruptStatus : u32 {
        const USED_RING_UPDATE = 1 << 0;
        const CONFIGURATION_CHANGE = 1 << 1;
    }
}

#[repr(C)]
struct Config {
    /// The guest_cid field contains the guest’s context ID, which uniquely identifies the device for its lifetime.
    /// The upper 32 bits of the CID are reserved and zeroed.
    guest_cid: ReadOnly<u64>,
}

// virtio 5.1.6 Device Operation
#[repr(C)]
#[derive(Debug, Default)]
struct Header {
    src_cid: Volatile<u64>,
    dst_cid: Volatile<u64>,
    src_port: Volatile<u32>,
    dst_port: Volatile<u32>,
    len: Volatile<u32>,
    type_: Volatile<Type>,
    op: Volatile<Op>,
    flags: Volatile<Flags>,
    buf_alloc: Volatile<u32>,
    fwd_cnt: Volatile<u32>,
}

unsafe impl AsBuf for Header {}

#[repr(u16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum Type {
    /// in-order, guaranteed, connection-oriented delivery without message boundaries
    VIRTIO_VSOCK_TYPE_STREAM = 1,
    /// in-order, guaranteed, connection-oriented delivery with message and record boundaries
    VIRTIO_VSOCK_TYPE_SEQPACKET = 2,
}

impl Default for Type {
    fn default() -> Self {
        Self::VIRTIO_VSOCK_TYPE_SEQPACKET
    }
}

#[repr(u16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum Op {
    VIRTIO_VSOCK_OP_INVALID = 0,
    /* Connect operations */
    VIRTIO_VSOCK_OP_REQUEST = 1,
    VIRTIO_VSOCK_OP_RESPONSE = 2,
    VIRTIO_VSOCK_OP_RST = 3,
    VIRTIO_VSOCK_OP_SHUTDOWN = 4,
    /// To send payload
    VIRTIO_VSOCK_OP_RW = 5,
    /// Tell the peer our credit info
    VIRTIO_VSOCK_OP_CREDIT_UPDATE = 6,
    /// Request the peer to send the credit info to us
    VIRTIO_VSOCK_OP_CREDIT_REQUEST = 7,
}

impl Default for Op {
    fn default() -> Self {
        Self::VIRTIO_VSOCK_OP_RW
    }
}

bitflags! {
    struct Flags : u32 {
        const VIRTIO_VSOCK_SEQ_EOM = 1 << 0;
        const VIRTIO_VSOCK_SEQ_EOR = 1 << 1;
    }
}

// virtio 5.10.2 Virtqueues
const QUEUE_RECEIVE: u16 = 0;
const QUEUE_TRANSMIT: u16 = 1;
const QUEUE_EVENT: u16 = 2;
