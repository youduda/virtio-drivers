use core::mem::{size_of, MaybeUninit};

use super::*;
use crate::transport::Transport;
use crate::volatile::{volread, ReadOnly, Volatile};
use bitflags::*;
use core::hint::spin_loop;
use log::*;

/// The virtio network device is a virtual ethernet card.
///
/// It has enhanced rapidly and demonstrates clearly how support for new
/// features are added to an existing device.
/// Empty buffers are placed in one virtqueue for receiving packets, and
/// outgoing packets are enqueued into another for transmission in that order.
/// A third command queue is used to control advanced filtering features.
pub struct VirtIONet<H: Hal, T: Transport> {
    transport: T,
    mac: EthernetAddress,
    recv_queue: VirtQueue<H>,
    send_queue: VirtQueue<H>,
}

impl<H: Hal, T: Transport> VirtIONet<H, T> {
    /// Create a new VirtIO-Net driver.
    pub fn new(mut transport: T) -> Result<Self> {
        transport.begin_init(|features| {
            let features = Features::from_bits_truncate(features);
            info!("Device features {:?}", features);
            let supported_features = Features::MAC | Features::STATUS;
            (features & supported_features).bits()
        });
        // read configuration space
        let config = transport.config_space::<Config>()?;
        let mac;
        // Safe because config points to a valid MMIO region for the config space.
        unsafe {
            mac = volread!(config, mac);
            debug!("Got MAC={:?}, status={:?}", mac, volread!(config, status));
        }

        let queue_num = 2; // for simplicity
        let recv_queue = VirtQueue::new(&mut transport, QUEUE_RECEIVE, queue_num)?;
        let send_queue = VirtQueue::new(&mut transport, QUEUE_TRANSMIT, queue_num)?;

        transport.finish_init();

        Ok(VirtIONet {
            transport,
            mac,
            recv_queue,
            send_queue,
        })
    }

    /// Acknowledge interrupt.
    pub fn ack_interrupt(&mut self) -> bool {
        self.transport.ack_interrupt()
    }

    /// Get MAC address.
    pub fn mac(&self) -> EthernetAddress {
        self.mac
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
    pub fn recv(&mut self, buf: &mut [u8]) -> Result<usize> {
        static mut header: Option<*mut Header> = None;
        if unsafe { header.is_none() } {
            let dma_page = H::dma_alloc(1);
            let vaddr_dma_page = H::phys_to_virt(dma_page);
            unsafe {
                header = Some(vaddr_dma_page as *mut Header);
            }
        }
        let header_buf = unsafe {
            (**header.as_ref().unwrap()).as_buf_mut()
        };

        self.recv_queue.add(&[], &[header_buf, buf])?;
        self.transport.notify(QUEUE_RECEIVE);
        while !self.recv_queue.can_pop() {
            spin_loop();
        }

        let (_, len) = self.recv_queue.pop_used()?;
        Ok(len as usize - size_of::<Header>())
    }

    /// Send a packet.
    pub fn send(&mut self, buf: &[u8]) -> Result {
        static mut header: Option<*mut Header> = None;
        if unsafe { header.is_none() } {
            let dma_page = H::dma_alloc(1);
            let vaddr_dma_page = H::phys_to_virt(dma_page);
            unsafe {
                header = Some(vaddr_dma_page as *mut Header);
            }
        }
        let header_buf = unsafe {
            (**header.as_ref().unwrap()).as_buf()
        };

        self.send_queue.add(&[header_buf, buf], &[])?;
        self.transport.notify(QUEUE_TRANSMIT);
        while !self.send_queue.can_pop() {
            spin_loop();
        }
        self.send_queue.pop_used()?;
        Ok(())
    }
}

bitflags! {
    struct Features: u64 {
        /// Device handles packets with partial checksum.
        /// This "checksum offload" is a common feature on modern network cards.
        const CSUM = 1 << 0;
        /// Driver handles packets with partial checksum.
        const GUEST_CSUM = 1 << 1;
        /// Control channel offloads reconfiguration support.
        const CTRL_GUEST_OFFLOADS = 1 << 2;
        /// Device maximum MTU reporting is supported.
        ///
        /// If offered by the device, device advises driver about the value of
        /// its maximum MTU. If negotiated, the driver uses mtu as the maximum
        /// MTU value.
        const MTU = 1 << 3;
        /// Device has given MAC address.
        const MAC = 1 << 5;
        /// Device handles packets with any GSO type. (legacy)
        const GSO = 1 << 6;
        /// Driver can receive TSOv4.
        const GUEST_TSO4 = 1 << 7;
        /// Driver can receive TSOv6.
        const GUEST_TSO6 = 1 << 8;
        /// Driver can receive TSO with ECN.
        const GUEST_ECN = 1 << 9;
        /// Driver can receive UFO.
        const GUEST_UFO = 1 << 10;
        /// Device can receive TSOv4.
        const HOST_TSO4 = 1 << 11;
        /// Device can receive TSOv6.
        const HOST_TSO6 = 1 << 12;
        /// Device can receive TSO with ECN.
        const HOST_ECN = 1 << 13;
        /// Device can receive UFO.
        const HOST_UFO = 1 << 14;
        /// Driver can merge receive buffers.
        const MRG_RXBUF = 1 << 15;
        /// Configuration status field is available.
        const STATUS = 1 << 16;
        /// Control channel is available.
        const CTRL_VQ = 1 << 17;
        /// Control channel RX mode support.
        const CTRL_RX = 1 << 18;
        /// Control channel VLAN filtering.
        const CTRL_VLAN = 1 << 19;
        ///
        const CTRL_RX_EXTRA = 1 << 20;
        /// Driver can send gratuitous packets.
        const GUEST_ANNOUNCE = 1 << 21;
        /// Device supports multiqueue with automatic receive steering.
        const MQ = 1 << 22;
        /// Set MAC address through control channel.
        const CTL_MAC_ADDR = 1 << 23;

        // device independent
        const RING_INDIRECT_DESC = 1 << 28;
        const RING_EVENT_IDX = 1 << 29;
        const VERSION_1 = 1 << 32; // legacy
    }
}

bitflags! {
    struct Status: u16 {
        const LINK_UP = 1;
        const ANNOUNCE = 2;
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
    mac: ReadOnly<EthernetAddress>,
    status: ReadOnly<Status>,
}

type EthernetAddress = [u8; 6];

// virtio 5.1.6 Device Operation
#[repr(C)]
#[derive(Debug)]
struct Header {
    flags: Volatile<Flags>,
    gso_type: Volatile<GsoType>,
    hdr_len: Volatile<u16>, // cannot rely on this
    gso_size: Volatile<u16>,
    csum_start: Volatile<u16>,
    csum_offset: Volatile<u16>,
    // payload starts from here
}

unsafe impl AsBuf for Header {}

bitflags! {
    struct Flags: u8 {
        const NEEDS_CSUM = 1;
        const DATA_VALID = 2;
        const RSC_INFO   = 4;
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum GsoType {
    NONE = 0,
    TCPV4 = 1,
    UDP = 3,
    TCPV6 = 4,
    ECN = 0x80,
}

const QUEUE_RECEIVE: u16 = 0;
const QUEUE_TRANSMIT: u16 = 1;
