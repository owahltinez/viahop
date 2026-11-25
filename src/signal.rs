//! Zero-dependency SIGINT/SIGTERM handling for graceful stop and checkpoint preservation.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;

static STOP_COUNT: AtomicUsize = AtomicUsize::new(0);
static GLOBAL_STOP: AtomicBool = AtomicBool::new(false);

/// Installs process-wide SIGINT/SIGTERM handlers. First signal requests a graceful stop;
/// a second signal terminates immediately.
pub fn install_signal_handlers() {
    STOP_COUNT.store(0, Ordering::SeqCst);
    GLOBAL_STOP.store(false, Ordering::SeqCst);
    #[cfg(unix)]
    unsafe {
        const SIGINT: i32 = 2;
        const SIGTERM: i32 = 15;
        unsafe extern "C" {
            fn signal(sig: i32, handler: extern "C" fn(i32)) -> usize;
        }
        signal(SIGINT, handle_unix_signal);
        signal(SIGTERM, handle_unix_signal);
    }
    #[cfg(windows)]
    unsafe {
        #[link(name = "kernel32")]
        unsafe extern "system" {
            fn SetConsoleCtrlHandler(
                handler: Option<unsafe extern "system" fn(u32) -> i32>,
                add: i32,
            ) -> i32;
        }
        SetConsoleCtrlHandler(Some(handle_windows_ctrl), 1);
    }
}

/// Returns true if a termination signal has been received.
#[inline]
pub fn stop_requested(extra: Option<&Arc<AtomicBool>>) -> bool {
    GLOBAL_STOP.load(Ordering::Relaxed)
        || extra.is_some_and(|flag| flag.load(Ordering::Relaxed))
}

/// Resets the global signal stop flag (intended for tests).
pub fn reset_global_stop() {
    STOP_COUNT.store(0, Ordering::SeqCst);
    GLOBAL_STOP.store(false, Ordering::SeqCst);
}

#[cfg(unix)]
extern "C" fn handle_unix_signal(_sig: i32) {
    let prev = STOP_COUNT.fetch_add(1, Ordering::SeqCst);
    GLOBAL_STOP.store(true, Ordering::SeqCst);
    if prev >= 1 {
        unsafe extern "C" {
            fn _exit(status: i32) -> !;
        }
        unsafe { _exit(130) };
    }
}

#[cfg(windows)]
unsafe extern "system" fn handle_windows_ctrl(_ctrl_type: u32) -> i32 {
    let prev = STOP_COUNT.fetch_add(1, Ordering::SeqCst);
    GLOBAL_STOP.store(true, Ordering::SeqCst);
    if prev >= 1 {
        std::process::exit(130);
    }
    1
}
