//! Thin C-ABI exports for C, Python (`ctypes`), and KiCad plugins.

use std::collections::HashSet;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int};
use std::ptr;

use crate::{route, write_ses, DsnDesign, RouterConfig};

static VERSION_CSTR: &[u8] = concat!(env!("CARGO_PKG_VERSION"), "\0").as_bytes();

/// Returns a static NUL-terminated version string.
#[no_mangle]
pub extern "C" fn viahop_version() -> *const c_char {
    VERSION_CSTR.as_ptr() as *const c_char
}

/// Frees a NUL-terminated string allocated by `viahop_route_str`.
///
/// # Safety
/// `ptr` must be NULL or a pointer previously returned by `viahop_route_str`.
#[no_mangle]
pub unsafe extern "C" fn viahop_free_str(ptr: *mut c_char) {
    if !ptr.is_null() {
        drop(unsafe { CString::from_raw(ptr) });
    }
}

/// Routes a Specctra DSN file on disk and writes the resulting SES file.
/// Returns 0 on success, non-zero on error.
///
/// # Safety
/// All non-null string pointers must be valid NUL-terminated C strings.
#[no_mangle]
pub unsafe extern "C" fn viahop_route_file(
    input_dsn_path: *const c_char,
    output_ses_path: *const c_char,
    max_passes: c_int,
    ignored_nets_csv: *const c_char,
    via_at_smd: c_int,
    threads: c_int,
    timeout_secs: c_int,
    err_buf: *mut c_char,
    err_buf_len: usize,
) -> c_int {
    let res = (|| -> Result<(), String> {
        let in_path = unsafe { cstr_req(input_dsn_path, "input_dsn_path") }?;
        let out_path = unsafe { cstr_req(output_ses_path, "output_ses_path") }?;
        let dsn_text = std::fs::read_to_string(&in_path)
            .map_err(|e| format!("failed to read {in_path}: {e}"))?;
        let design = DsnDesign::parse(&dsn_text)?;
        let cfg = build_config(
            max_passes,
            unsafe { cstr_opt(ignored_nets_csv) },
            via_at_smd,
            threads,
            timeout_secs,
        );
        let result = route(&design, &cfg)?;
        let ses_text = write_ses(&design, &result);
        std::fs::write(&out_path, ses_text)
            .map_err(|e| format!("failed to write {out_path}: {e}"))?;
        Ok(())
    })();

    match res {
        Ok(()) => 0,
        Err(msg) => {
            unsafe { write_err_buf(err_buf, err_buf_len, &msg) };
            1
        }
    }
}

/// Routes an in-memory Specctra DSN string and returns heap-allocated SES and JSON summary strings.
///
/// # Safety
/// `dsn_str` must be a valid NUL-terminated C string. Output pointers may be NULL.
#[no_mangle]
pub unsafe extern "C" fn viahop_route_str(
    dsn_str: *const c_char,
    max_passes: c_int,
    ignored_nets_csv: *const c_char,
    via_at_smd: c_int,
    threads: c_int,
    timeout_secs: c_int,
    out_ses: *mut *mut c_char,
    out_summary_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    let res = (|| -> Result<(String, String), String> {
        let src = unsafe { cstr_req(dsn_str, "dsn_str") }?;
        let design = DsnDesign::parse(&src)?;
        let cfg = build_config(
            max_passes,
            unsafe { cstr_opt(ignored_nets_csv) },
            via_at_smd,
            threads,
            timeout_secs,
        );
        let result = route(&design, &cfg)?;
        let ses_text = write_ses(&design, &result);
        let json = result.stats.to_json();
        Ok((ses_text, json))
    })();

    match res {
        Ok((ses, json)) => {
            if !out_ses.is_null() {
                unsafe { *out_ses = into_c_ptr(&ses) };
            }
            if !out_summary_json.is_null() {
                unsafe { *out_summary_json = into_c_ptr(&json) };
            }
            if !out_err.is_null() {
                unsafe { *out_err = ptr::null_mut() };
            }
            0
        }
        Err(msg) => {
            if !out_ses.is_null() {
                unsafe { *out_ses = ptr::null_mut() };
            }
            if !out_summary_json.is_null() {
                unsafe { *out_summary_json = ptr::null_mut() };
            }
            if !out_err.is_null() {
                unsafe { *out_err = into_c_ptr(&msg) };
            }
            1
        }
    }
}

fn build_config(
    max_passes: c_int,
    ignored_csv: Option<String>,
    via_at_smd: c_int,
    threads: c_int,
    timeout_secs: c_int,
) -> RouterConfig {
    let mut ignored_nets = HashSet::new();
    if let Some(csv) = ignored_csv {
        for tok in csv.split(',') {
            let t = tok.trim();
            if !t.is_empty() {
                ignored_nets.insert(t.to_string());
            }
        }
    }
    RouterConfig {
        max_passes: if max_passes > 0 { max_passes as usize } else { 20 },
        ignored_nets,
        via_at_smd: if via_at_smd > 0 {
            Some(true)
        } else if via_at_smd < 0 {
            None
        } else {
            Some(false)
        },
        threads: if threads > 0 { threads as usize } else { 1 },
        timeout_secs: if timeout_secs > 0 {
            Some(timeout_secs as u64)
        } else {
            None
        },
        ..Default::default()
    }
}

unsafe fn cstr_req(ptr: *const c_char, name: &str) -> Result<String, String> {
    if ptr.is_null() {
        return Err(format!("{name} must not be NULL"));
    }
    unsafe { CStr::from_ptr(ptr) }
        .to_str()
        .map(ToString::to_string)
        .map_err(|e| format!("{name} is not valid UTF-8: {e}"))
}

unsafe fn cstr_opt(ptr: *const c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    unsafe { CStr::from_ptr(ptr) }
        .to_str()
        .ok()
        .map(ToString::to_string)
}

fn into_c_ptr(s: &str) -> *mut c_char {
    CString::new(s)
        .unwrap_or_else(|_| CString::new(s.replace('\0', " ")).unwrap())
        .into_raw()
}

unsafe fn write_err_buf(buf: *mut c_char, len: usize, msg: &str) {
    if buf.is_null() || len == 0 {
        return;
    }
    let bytes = msg.as_bytes();
    let copy_len = bytes.len().min(len - 1);
    unsafe {
        ptr::copy_nonoverlapping(bytes.as_ptr(), buf as *mut u8, copy_len);
        *buf.add(copy_len) = 0;
    }
}
