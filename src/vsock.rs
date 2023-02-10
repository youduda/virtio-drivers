use core::mem::{size_of, MaybeUninit};

use super::*;
use crate::transport::Transport;
use crate::volatile::{volread, volwrite, ReadOnly, Volatile};
use bitflags::*;
use core::hint::spin_loop;
use core::ptr::NonNull;
use log::*;

// TODO: ( PAGE_SIZE - size_of::<Header>() * 2 ) / 2
pub const SEND_RECV_DATA_BUF_SIZE: usize = 0x700;

#[repr(transparent)]
struct DataBuf {
    data: [u8; SEND_RECV_DATA_BUF_SIZE],
}

unsafe impl AsBuf for DataBuf {}

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
    send_header_buf: NonNull<Header>,
    recv_header_buf: NonNull<Header>,
    send_data_buf: NonNull<DataBuf>,
    recv_data_buf: NonNull<DataBuf>,
    fwd_cnt: u32,
    queue_filled: bool,
}

impl<H: Hal, T: Transport> VirtIOVsock<H, T> {
    /// Create a new VirtIO-VSock driver.
    pub fn new(mut transport: T) -> Result<Self> {
        transport.begin_init(|features| {
            let features = Features::from_bits_truncate(features);
            info!("Device features {:?}", features);
            let supported_features = Features::VIRTIO_VSOCK_F_STREAM;
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
        let recv_data_buf =
            NonNull::new(vaddr_dma_page as *mut DataBuf).expect("Allocation of data buffer failed");
        let send_data_buf = NonNull::new((vaddr_dma_page + size_of::<DataBuf>()) as *mut DataBuf)
            .expect("Allocation of data buffer failed");
        let send_header_buf =
            NonNull::new((vaddr_dma_page + size_of::<DataBuf>() * 2) as *mut Header)
                .expect("Allocation of header buffer failed");
        let recv_header_buf = NonNull::new(
            (vaddr_dma_page + size_of::<DataBuf>() * 2 + size_of::<Header>()) as *mut Header,
        )
        .expect("Allocation of header buffer failed");

        Ok(VirtIOVsock {
            transport,
            guest_cid,
            recv_queue,
            send_queue,
            event_queue,
            send_header_buf,
            recv_header_buf,
            send_data_buf,
            recv_data_buf,
            fwd_cnt: 0,
            queue_filled: false,
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

    fn recv_init(&mut self) -> Result {
        if self.queue_filled {
            return Ok(());
        }
        self.queue_filled = true;
        // Note: `header_buf` and `self.header_buf` point to the same memory address. Same for `recv_data_buf`
        let recv_header_buf = unsafe { self.recv_header_buf.as_mut().as_buf_mut() };
        let recv_data_buf = unsafe { self.recv_data_buf.as_mut().as_buf_mut() };

        self.recv_queue
            .add(&[], &[recv_header_buf, recv_data_buf])?;
        self.transport.notify(QUEUE_RECEIVE);
        Ok(())
    }

    /// Receive a packet.
    /// Returns a tuple (src_cid, src_port, dst_port, buf_len)
    fn recv_frame(&mut self) -> Result<(u64, u32, u32, usize)> {
        assert!(self.queue_filled);
        self.queue_filled = false;
        while !self.recv_queue.can_pop() {
            spin_loop();
        }
        let (_, len) = self.recv_queue.pop_used()?;
        self.fwd_cnt += len;

        let dst_cid = unsafe { volread!(self.recv_header_buf, dst_cid) };
        assert_eq!(
            dst_cid, self.guest_cid,
            "TODO: What to to if received frame has different dst_cid"
        );
        // TODO: virtio 5.10.6.3: A VIRTIO_VSOCK_OP_RST reply MUST be sent if a packet is received with an unknown type

        Ok((
            unsafe { volread!(self.recv_header_buf, src_cid) },
            unsafe { volread!(self.recv_header_buf, src_port) },
            unsafe { volread!(self.recv_header_buf, dst_port) },
            len as usize - size_of::<Header>(),
        ))
    }

    /// Send a packet.
    fn send_int(&mut self, src_port: u32, dst_cid: u64, dst_port: u32, buf_len: usize) -> Result {
        // Note: `header_buf` and `self.header_buf` point to the same memory address
        let header_buf = unsafe { self.send_header_buf.as_mut().as_buf_mut() };

        unsafe {
            volwrite!(self.send_header_buf, src_cid, self.guest_cid);
            volwrite!(self.send_header_buf, src_port, src_port);
            volwrite!(self.send_header_buf, dst_cid, dst_cid);
            volwrite!(self.send_header_buf, dst_port, dst_port);

            volwrite!(self.send_header_buf, len, buf_len as u32);
            volwrite!(self.send_header_buf, type_, Type::VIRTIO_VSOCK_TYPE_STREAM);
            volwrite!(self.send_header_buf, flags, Flags::empty());

            volwrite!(self.send_header_buf, buf_alloc, 0x1000);
            volwrite!(self.send_header_buf, fwd_cnt, self.fwd_cnt);
        }

        let send_data_buf = unsafe { self.send_data_buf.as_ref().as_buf() };
        self.send_queue.add(&[header_buf, send_data_buf], &[])?;
        self.transport.notify(QUEUE_TRANSMIT);
        while !self.send_queue.can_pop() {
            spin_loop();
        }
        self.send_queue.pop_used()?;
        Ok(())
    }

    pub fn send(&mut self, src_port: u32, dst_cid: u64, dst_port: u32, buf: &[u8]) -> Result {
        let send_data_buf = unsafe { self.send_data_buf.as_mut().as_buf_mut() };
        debug_assert!(buf.len() <= send_data_buf.len());
        send_data_buf[..buf.len()].copy_from_slice(buf);
        unsafe {
            volwrite!(self.send_header_buf, op, Op::VIRTIO_VSOCK_OP_RW);
        }

        self.send_int(src_port, dst_cid, dst_port, buf.len())
    }

    pub fn recv_event() {
        // virtio 5.10.6.7
        todo!()
    }

    /// Listen for a new incoming connection
    fn accept(&mut self, src_cid: u64, src_port: u32, dst_port: u32) -> Result {
        trace!("Accepting connection");

        let recv_data_buf = unsafe { self.recv_data_buf.as_mut().as_buf_mut() };

        let frame_type = unsafe { volread!(self.recv_header_buf, type_) };
        debug_assert_eq!(frame_type, Type::VIRTIO_VSOCK_TYPE_STREAM);
        let frame_op = unsafe { volread!(self.recv_header_buf, op) };
        debug_assert_eq!(frame_op, Op::VIRTIO_VSOCK_OP_REQUEST);
        trace!("Received connection request");

        unsafe {
            volwrite!(self.send_header_buf, op, Op::VIRTIO_VSOCK_OP_RESPONSE);
        }
        self.send_int(dst_port, src_cid, src_port, 0)?;

        trace!("Accepted connection");

        Ok(())
    }

    /// Accepts incoming connections, receives incoming data
    /// Returns a tuple (operation, src_cid, src_port, dst_port, buf_len)
    pub fn handle_next(
        &mut self,
        buf: &mut [u8],
        blocking: bool,
    ) -> Result<(VSockOp, u64, u32, u32, usize)> {
        let recv_data_buf = unsafe { self.recv_data_buf.as_ref().as_buf() };
        debug_assert!(buf.len() >= recv_data_buf.len());

        self.recv_init()?;
        if !blocking && !self.can_recv() {
            return Ok((VSockOp::WouldBlock, 0, 0, 0, 0));
        }
        let (src_cid, src_port, dst_port, buf_len) = self.recv_frame()?;
        let op = unsafe { volread!(self.recv_header_buf, op) };
        let type_ = unsafe { volread!(self.recv_header_buf, type_) };

        match op {
            Op::VIRTIO_VSOCK_OP_RW => {
                buf[..buf_len].copy_from_slice(&recv_data_buf[..buf_len]);
                Ok((VSockOp::DataFrame, src_cid, src_port, dst_port, buf_len))
            }
            Op::VIRTIO_VSOCK_OP_REQUEST => {
                self.accept(src_cid, src_port, dst_port);
                Ok((VSockOp::Accepted, src_cid, src_port, dst_port, 0))
            }
            _ => todo!(),
        }
    }
}

pub enum VSockOp {
    Accepted,
    DataFrame,
    WouldBlock,
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
// TODO: `packed` should not be needed. Looks like a compiler bug or other changes in newer Rust versions fixing this
#[repr(C, packed)]
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

impl Default for Flags {
    fn default() -> Self {
        Self::empty()
    }
}

// virtio 5.10.2 Virtqueues
const QUEUE_RECEIVE: u16 = 0;
const QUEUE_TRANSMIT: u16 = 1;
const QUEUE_EVENT: u16 = 2;
