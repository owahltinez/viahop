//! `viahop`: Fast, standalone, zero-dependency headless Specctra DSN/SES multi-layer PCB autorouter.

pub(crate) mod astar;
pub mod board;
pub mod connectivity;
pub mod drc;
pub mod dsn;
pub mod fanout;
pub mod ffi;
pub mod geometry;
pub mod grid;
pub mod optimize;
pub mod preflight;
pub mod report;
pub mod router;
pub mod ses;
pub mod sexp;
pub mod signal;

pub use board::{
    Board, BoardOptions, FixedPlane, NetRoute, RoutedSegment, RoutedVia, StaticBlocker,
};
pub use connectivity::RouteScore;
pub use drc::{DrcReport, DrcViolation, check_drc};
pub use dsn::DsnDesign;
pub use fanout::{BgaCluster, FanoutPlan, PadFanout, analyze_fanout};
pub use preflight::{
    BgaClusterSummary, BlockedNetPair, FixedViolation, PreflightReport, SealedPadInfo,
    run_preflight, run_preflight_with_options,
};
pub use report::{RouteReport, UnroutedConnection, UnroutedKind, ViolationDetail};
pub use router::{RouteStats, RouterConfig, route_board, route_board_with_checkpoint};
pub use ses::parse_ses;

/// Complete result of routing a parsed Specctra DSN design.
#[derive(Debug, Clone)]
pub struct RouteResult {
    pub board: Board,
    pub routes: Vec<NetRoute>,
    pub stats: RouteStats,
}

impl RouteStats {
    pub fn to_json(&self) -> String {
        format!(
            "{{\"total_nets\":{},\"routable_nets\":{},\"routed_nets\":{},\"unconnected_pairs\":{},\"total_segments\":{},\"total_vias\":{},\"total_wirelength_um\":{:.1},\"passes_run\":{},\"starts_run\":{},\"drc_violations\":{},\"dense_bga_pads\":{},\"pofv_capable_pads\":{},\"grid_step_um\":{:.1},\"stopped_early\":{},\"elapsed_ms\":{}}}",
            self.total_nets,
            self.routable_nets,
            self.routed_nets,
            self.unconnected_pairs,
            self.total_segments,
            self.total_vias,
            self.total_wirelength_um,
            self.passes_run,
            self.starts_run,
            self.drc_violations,
            self.dense_bga_pads,
            self.pofv_capable_pads,
            self.grid_step_um,
            self.stopped_early,
            self.elapsed_ms,
        )
    }
}

/// Resolves `design` into a physical board and routes all non-ignored nets.
pub fn route(design: &DsnDesign, config: &RouterConfig) -> Result<RouteResult, String> {
    let options = BoardOptions {
        hole_to_hole_um: config.hole_to_hole_um,
        edge_clearance_um: config.edge_clearance_um,
    };
    let board =
        Board::from_dsn_with_options(design, &config.ignored_nets, config.via_at_smd, options)?;
    let inner = router::route_board(&board, config);
    Ok(RouteResult {
        board,
        routes: inner.routes,
        stats: inner.stats,
    })
}

/// Resolves `design` into a physical board and routes with a checkpoint callback invoked
/// whenever a strictly better routing solution is found.
pub fn route_with_checkpoint<F>(
    design: &DsnDesign,
    config: &RouterConfig,
    on_best: F,
) -> Result<RouteResult, String>
where
    F: Fn(&Board, &[NetRoute], RouteScore) + Sync,
{
    let options = BoardOptions {
        hole_to_hole_um: config.hole_to_hole_um,
        edge_clearance_um: config.edge_clearance_um,
    };
    let board =
        Board::from_dsn_with_options(design, &config.ignored_nets, config.via_at_smd, options)?;
    let hook = |routes: &[NetRoute], score: RouteScore| on_best(&board, routes, score);
    let inner = router::route_board_with_checkpoint(&board, config, Some(&hook));
    Ok(RouteResult {
        board,
        routes: inner.routes,
        stats: inner.stats,
    })
}

/// Serializes `result` into a KiCad-10-compatible Specctra `.ses` document.
pub fn write_ses(design: &DsnDesign, result: &RouteResult) -> String {
    ses::write_ses(design, &result.board, &result.routes)
}
