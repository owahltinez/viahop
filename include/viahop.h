#ifndef VIAHOP_H
#define VIAHOP_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns a static NUL-terminated version string (do not free). */
const char *viahop_version(void);

/*
 * Route a Specctra DSN file on disk and write a Specctra SES file.
 * Returns 0 on success, non-zero on error.
 * Optional string args (ignored_nets_csv) may be NULL or empty.
 * via_at_smd: >0 force on, 0 force off, <0 inherit from DSN.
 * If err_buf is non-NULL and err_buf_len > 0, writes error message on failure.
 */
int viahop_route_file(
    const char *input_dsn_path,
    const char *output_ses_path,
    int max_passes,
    const char *ignored_nets_csv,
    int via_at_smd,
    int threads,
    int timeout_secs,
    char *err_buf,
    size_t err_buf_len);

/*
 * Route an in-memory Specctra DSN string.
 * On success (return 0), *out_ses (and optionally *out_summary_json) receive
 * heap-allocated NUL-terminated strings that MUST be freed via viahop_free_str().
 * On failure (return non-zero), *out_err (if non-NULL) receives an error string
 * that MUST be freed via viahop_free_str().
 */
int viahop_route_str(
    const char *dsn_str,
    int max_passes,
    const char *ignored_nets_csv,
    int via_at_smd,
    int threads,
    int timeout_secs,
    char **out_ses,
    char **out_summary_json,
    char **out_err);

/* Free a string allocated by viahop_route_str(). Safe to call with NULL. */
void viahop_free_str(char *ptr);

#ifdef __cplusplus
}
#endif

#endif /* VIAHOP_H */
