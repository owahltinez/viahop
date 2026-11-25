# viahop

Fast, standalone, zero-dependency, MIT-licensed headless Specctra DSN/SES multi-layer PCB autorouter in pure Rust (no JVM or X11).

## Features

- **Freerouting-compatible CLI + modern flags**: drop-in headless invocation (`-de`, `-do`, `-mp`, `-inc`, `-mt`, `-dr`, `-dl`, `--via-at-smd`, `--timeout`/`--max-time`, `--json-summary`) plus `--multi-start`, `--initial-session`, `--report`, and `--diagnose`.
- **Rust library + C-ABI (`rlib` + `cdylib`)**: direct in-process integration from Rust, C/C++, Python `ctypes`, or KiCad plugins (`include/viahop.h`).
- **Parallel multi-start + bad-pass undo**: start 0 runs canonical ordering with zero overhead when clean; if unrouted pairs or DRC issues remain, shuffled seeds run in parallel and keep the best board while undoing degrading passes.
- **Atomic checkpointing & graceful stop**: `-do` is updated atomically whenever a better solution is found; `Ctrl+C` (`SIGINT`/`SIGTERM`) or `--max-time` stops cleanly and keeps the best routed `.ses` on disk (a second signal exits immediately).
- **Initial-session resume (`--initial-session`)**: load an existing `.ses` checkpoint, preserve clean routed nets, finish remaining connections, and run post-route optimization.
- **Structured JSON reports & diagnosis (`--report`, `--diagnose`)**: report every unrouted pair and DRC violation, and test-route each unrouted pair alone on an empty grid to classify `congestion` vs static `blocked` geometry/rules.
- **Pull-tight & via-reduction optimizer**: spatially indexed multi-segment octilinear shortcutting, inter-via layer flattening (via-pair elimination), collinear merging, redundant via removal, and dangling stub pruning gated by exact continuous DRC.
- **Full KiCad 10 Specctra DSN/SES + POFV BGA fanout**: handles `um`/`mil`/`mm` resolutions, signal vs power layers, `(plane ...)` zones, board boundary polygons, structure and image keepouts, front/back placement mirroring and rotation, fixed wires/vias, per-class clearances, and centered POFV via-in-pad fanout for dense `0.40 mm` / `0.50 mm` BGAs.

## Build & Test

```bash
cargo build --release
cargo test
cargo clippy --all-targets -- -D warnings
```

Outputs:
- CLI binary: `target/release/viahop`
- Shared library: `target/release/libviahop.so` (C header: `include/viahop.h`)

## CLI Usage

```bash
# Standard Freerouting-compatible invocation
viahop -de board.dsn -do board.ses -mp 25 -inc GND,AGND --via-at-smd -mt 4

# Parallel multi-start with diagnostic report and time cap
viahop -de board.dsn -do board.ses -mt 4 --multi-start=4 --max-time=60 --report report.json --diagnose

# Resume and optimize from an existing session checkpoint
viahop -de board.dsn -do board_opt.ses --initial-session board.ses -mp 10
```

Flags:
- `-de`, `--input <FILE>`: input Specctra DSN
- `-do`, `--output <FILE>`: output Specctra SES (atomically checkpointed on improvement)
- `-mp`, `--max-passes <N>`: max rip-up-and-reroute passes per start (default `20`)
- `--multi-start <N>`: parallel shuffled starts if unrouted pairs or DRC issues remain (default `4`)
- `--initial-session <FILE>`: resume routing/optimization from an existing `.ses` file
- `--report <FILE>`: write detailed JSON report (stats, unrouted pairs, DRC violations)
- `--diagnose`: with `--report`, classify unrouted pairs as `congestion` vs `blocked`
- `-inc <LIST>`: comma-separated ignored net classes or net names
- `-mt`, `--threads <N>`: worker threads (default `1`)
- `-dr`, `--rules <FILE>`: optional Specctra rules file
- `-dl`, `--disable-log`: suppress stderr summary
- `--via-at-smd` / `--no-via-at-smd`: override DSN POFV via-in-pad setting
- `--timeout`, `--max-time <SECS>`: soft timeout in seconds
- `--json-summary`: emit JSON metrics to stdout
- `-h`/`--help`, `-V`/`--version`

## Rust & C-ABI Usage

Rust:
```rust
use viahop::{route, write_ses, DsnDesign, RouterConfig};

let design = DsnDesign::parse(&std::fs::read_to_string("board.dsn")?)?;
let result = route(&design, &RouterConfig::default())?;
std::fs::write("board.ses", write_ses(&design, &result))?;
```

C / Python (`include/viahop.h`):
```c
#include "viahop.h"

char err[256];
int rc = viahop_route_file(
    "board.dsn", "board.ses", 25, "GND,AGND", 1, 4, 60, err, sizeof(err));
```
