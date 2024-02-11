//! [`defmt`](https://github.com/knurling-rs/defmt) global logger over RTT.
//!
//! NOTE when using this crate it's not possible to use (link to) the `rtt-target` crate
//!
//! To use this crate, link to it by importing it somewhere in your project.
//!
//! ```
//! // src/main.rs or src/bin/my-app.rs
//! use defmt_rtt as _;
//! ```
//!
//! # Blocking/Non-blocking
//!
//! `probe-run` puts RTT into blocking-mode, to avoid losing data.
//!
//! As an effect this implementation may block forever if `probe-run` disconnects on runtime. This
//! is because the RTT buffer will fill up and writing will eventually halt the program execution.
//!
//! `defmt::flush` would also block forever in that case.
//!
//! # Critical section implementation
//!
//! This crate uses [`critical-section`](https://github.com/rust-embedded/critical-section) to ensure only one thread
//! is writing to the buffer at a time. You must import a crate that provides a `critical-section` implementation
//! suitable for the current target. See the `critical-section` README for details.
//!
//! For example, for single-core privileged-mode Cortex-M targets, you can add the following to your Cargo.toml.
//!
//! ```toml
//! [dependencies]
//! cortex-m = { version = "0.7.6", features = ["critical-section-single-core"]}
//! ```

#![no_std]

mod channel;
mod consts;

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::{channel::Channel, consts::BUF_SIZE};

#[defmt::global_logger]
struct Logger;

/// Global logger lock.
static TAKEN: [AtomicBool; 2] = [AtomicBool::new(false), AtomicBool::new(false)];
//static mut CS_RESTORE: [critical_section::RestoreState; 2] = [critical_section::RestoreState::invalid(), critical_section::RestoreState::invalid()];
static mut ENCODER: [defmt::Encoder; 2] = [defmt::Encoder::new(), defmt::Encoder::new()];

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let core = rp_pac::SIO.cpuid().read() as usize;

        // safety: Must be paired with corresponding call to release(), see below
        //let restore = unsafe { critical_section::acquire() };

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        if TAKEN[core].load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        TAKEN[core].store(true, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        //unsafe { CS_RESTORE[core] = restore };

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe { ENCODER[core].start_frame(make_do_write(core)) }
    }

    unsafe fn flush() {
        let core = rp_pac::SIO.cpuid().read() as usize;

        // safety: accessing the `&'static _` is OK because we have acquired a critical section.
        handle(core).flush();
    }

    unsafe fn release() {
        let core = rp_pac::SIO.cpuid().read() as usize;

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        ENCODER[core].end_frame(make_do_write(core));

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        TAKEN[core].store(false, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        //let restore = CS_RESTORE[core];

        // safety: Must be paired with corresponding call to acquire(), see above
        //critical_section::release(restore);
    }

    unsafe fn write(bytes: &[u8]) {
        let core = rp_pac::SIO.cpuid().read() as usize;

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        ENCODER[core].write(bytes, make_do_write(core));
    }
}

fn make_do_write(core: usize) -> impl FnMut(&[u8]) {
    move |bytes| { unsafe { handle(core).write_all(bytes) } }
}

#[repr(C)]
struct Header {
    id: [u8; 16],
    max_up_channels: usize,
    max_down_channels: usize,
    up_channels: [Channel; 2],
}

const MODE_MASK: usize = 0b11;
/// Block the application if the RTT buffer is full, wait for the host to read data.
const MODE_BLOCK_IF_FULL: usize = 2;
/// Don't block if the RTT buffer is full. Truncate data to output as much as fits.
const MODE_NON_BLOCKING_TRIM: usize = 1;

// make sure we only get shared references to the header/channel (avoid UB)
/// # Safety
/// `Channel` API is not re-entrant; this handle should not be held from different execution
/// contexts (e.g. thread-mode, interrupt context)
unsafe fn handle(core: usize) -> &'static Channel {
    // NOTE the `rtt-target` API is too permissive. It allows writing arbitrary data to any
    // channel (`set_print_channel` + `rprint*`) and that can corrupt defmt log frames.
    // So we declare the RTT control block here and make it impossible to use `rtt-target` together
    // with this crate.
    #[no_mangle]
    static mut _SEGGER_RTT: Header = Header {
        id: *b"SEGGER RTT\0\0\0\0\0\0",
        max_up_channels: 2,
        max_down_channels: 0,
        up_channels: [
            Channel {
                name: &NAME0 as *const _ as *const u8,
                buffer: unsafe { &mut BUFFER0 as *mut _ as *mut u8 },
                size: BUF_SIZE,
                write: AtomicUsize::new(0),
                read: AtomicUsize::new(0),
                flags: AtomicUsize::new(MODE_NON_BLOCKING_TRIM),
            },
            Channel {
                name: &NAME1 as *const _ as *const u8,
                buffer: unsafe { &mut BUFFER1 as *mut _ as *mut u8 },
                size: BUF_SIZE,
                write: AtomicUsize::new(0),
                read: AtomicUsize::new(0),
                flags: AtomicUsize::new(MODE_NON_BLOCKING_TRIM),
            },
        ],
    };

    #[cfg_attr(target_os = "macos", link_section = ".uninit,defmt-rtt.BUFFER")]
    #[cfg_attr(not(target_os = "macos"), link_section = ".uninit.defmt-rtt.BUFFER")]
    static mut BUFFER0: [u8; BUF_SIZE] = [0; BUF_SIZE];
    #[cfg_attr(target_os = "macos", link_section = ".uninit,defmt-rtt.BUFFER")]
    #[cfg_attr(not(target_os = "macos"), link_section = ".uninit.defmt-rtt.BUFFER")]
    static mut BUFFER1: [u8; BUF_SIZE] = [0; BUF_SIZE];

    // Place NAME in data section, so the whole RTT header can be read from RAM.
    // This is useful if flash access gets disabled by the firmware at runtime.
    #[link_section = ".data"]
    static NAME0: [u8; 7] = *b"defmt0\0";
    #[link_section = ".data"]
    static NAME1: [u8; 7] = *b"defmt1\0";

    &_SEGGER_RTT.up_channels[core]
}
