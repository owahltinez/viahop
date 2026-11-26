# viahop

Fast, standalone, zero-dependency, MIT-licensed headless Specctra DSN/SES multi-layer PCB autorouter in pure Rust (no JVM or X11).

## Features

- **Freerouting-compatible CLI + modern flags**: drop-in headless invocation (`-de`, `-do`, `-mp`, `-inc`, `-mt`, `-dr`, `-dl`, `--via-at-smd`, `--timeout`/`--max-time`, `--json-summary`) plus `--multi-start`, `--initial-session`, `--max-length`, `--match-group`, `--report`, and `--diagnose`.
- **Length & skew constraints (`--max-length`, `--match-group`)**: prioritize length-budgeted nets (including `+`-chained series-resistor nets) and insert DRC-gated octilinear trombone meanders post-optimization to match bus and differential-pair lengths.
- **Rust library + C-ABI (`rlib` + `cdylib`)**: direct in-process integration from Rust, C/C++, Python `ctypes`, or KiCad plugins (`include/viahop.h`).
- **Parallel multi-start + bad-pass undo**: start 0 runs canonical ordering with zero overhead when clean; if unrouted pairs or DRC issues remain, shuffled seeds run in parallel and keep the best board while undoing degrading passes.
- **Atomic checkpointing & graceful stop**: `-do` is updated atomically whenever a better solution is found; `Ctrl+C` (`SIGINT`/`SIGTERM`) or `--max-time` stops cleanly and keeps the best routed `.ses` on disk (a second signal exits immediately).
- **Initial-session resume (`--initial-session`)**: load an existing `.ses` checkpoint, preserve clean routed nets, finish remaining connections, and run post-route optimization.
- **Structured JSON reports & diagnosis (`--report`, `--diagnose`)**: report every unrouted pair and DRC violation, and test-route each unrouted pair alone on an empty grid to classify `congestion` vs static `blocked` geometry/rules.
- **Pull-tight & via-reduction optimizer**: spatially indexed multi-segment octilinear shortcutting, inter-via layer flattening (via-pair elimination), collinear merging, redundant via removal, and dangling stub pruning gated by exact continuous DRC.
- **Full KiCad 10 Specctra DSN/SES + POFV BGA fanout**: handles `um`/`mil`/`mm` resolutions, signal vs power layers, `(plane ...)` zones, board boundary polygons, structure and image keepouts, front/back placement mirroring and rotation, fixed wires/vias, per-class clearances, and centered POFV via-in-pad fanout for dense `0.40 mm` / `0.50 mm` BGAs.

## Install & Build

```bash
# Install from crates.io
cargo install viahop

# Or build from source
cargo build --release
cargo test
cargo clippy --all-targets -- -D warnings
```

Prebuilt CLI binaries, shared libraries (`libviahop.{so,dylib,dll}`), and `include/viahop.h` are also available on [GitHub Releases](https://github.com/owahltinez/viahop/releases).

Outputs:
- CLI binary: `target/release/viahop`
- Shared library: `target/release/libviahop.so` (C header: `include/viahop.h`)

## CLI Usage

```bash
# Clean CLI (also accepts Freerouting flags: -de, -do, -mp, -inc, -mt, -dr, -dl)
viahop -i board.dsn -o board.ses --max-passes 25 --ignore GND,AGND --via-at-smd --threads 4

# Parallel multi-start with diagnostic report and time cap
viahop -i board.dsn -o board.ses --threads 4 --multi-start=4 --max-time=60 --report report.json --diagnose

# Resume and optimize from an existing session checkpoint
viahop -i board.dsn -o board_opt.ses --initial-session board.ses --max-passes 10

# Enforce max-length budgets and matched-length bus/diff-pair skew tuning
viahop -i board.dsn -o board.ses \
  --max-length /PDM_CLK+/Audio/PDM_CLK_MIC=40mm \
  --match-group QSPI:CLK,IO0,IO1,IO2:0.5mm \
  --match-group DP:USB_DP,USB_DM:0.1mm@15mm \
  --report report.json
```

Flags:
- `-i`, `--input <FILE>` (alias: `-de`): input Specctra DSN
- `-o`, `--output <FILE>` (alias: `-do`): output Specctra SES (atomically checkpointed on improvement)
- `--max-passes <N>` (alias: `-mp`): max rip-up-and-reroute passes per start (default `20`)
- `--multi-start <N>`: parallel shuffled starts if unrouted pairs or DRC issues remain (default `4`)
- `--initial-session <FILE>`: resume routing/optimization from an existing `.ses` file
- `--max-length <SPEC>`: repeatable max length constraint `NET[+NET2...]=LEN` (`mm`/`um`/`mil`, default `mm`)
- `--match-group <SPEC>`: repeatable skew group `NAME:NET1,NET2,...:TOL[@TARGET]` with DRC-gated meander tuning
- `--report <FILE>`: write detailed JSON report (stats, unrouted pairs, DRC violations, length checks)
- `--diagnose`: with `--report`, classify unrouted pairs as `congestion` vs `blocked`
- `--ignore <LIST>` (aliases: `-inc`, `--ignore-net-classes`): comma-separated ignored net classes or net names
- `--threads <N>` (alias: `-mt`): worker threads (default `1`)
- `--rules <FILE>` (alias: `-dr`): optional Specctra rules file
- `-q`, `--quiet` (aliases: `-dl`, `--disable-log`): suppress stderr summary
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
