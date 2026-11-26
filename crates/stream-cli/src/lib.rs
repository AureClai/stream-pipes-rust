use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::path::{Path, PathBuf};
use std::time::Instant;
use serde::Serialize;
use stream_core::api::{collect, compile as compile_inputs, RunOptions, ScenarioInputs};
use stream_core::assignment::assign_demand;
use stream_core::benchmarks::run_benchmarks;
use stream_core::io::{compile_with_patches, load_scenario};
use stream_core::model::Scenario;
use stream_core::patch::{apply_patches_to_scenario, Patch};
use stream_core::reference::{compare_with_observations, ObservedRecord, ReferenceReport};
use stream_core::simulation::{MergeRule, Simulation};
use stream_core::validation::Validate;
use stream_core::verification::{verify_scenario, CheckStatus, VerificationReport};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run a simulation from a compiled scenario JSON or a scenario directory
    Run {
        /// Path to a compiled scenario JSON file or a scenario directory
        #[arg(short, long, value_name = "PATH")]
        input: PathBuf,

        /// Path to output results JSON file (optional)
        #[arg(short, long, value_name = "FILE")]
        output: Option<PathBuf>,

        /// Run assignment (generate & route vehicles from demand) before simulation
        #[arg(short, long)]
        assignment: bool,

        /// Merge rule at nodes: "in_order" (default, or from config.json in dir mode) or "capacity"
        #[arg(long)]
        merge: Option<String>,

        /// Aggregation bin size in seconds for link/pipe statistics and CVCs
        #[arg(long, default_value_t = 60.0)]
        bin_size: f64,

        /// Omit per-vehicle trajectories from the JSON output (for large networks)
        #[arg(long)]
        no_vehicles: bool,

        /// Patch JSON files (or patch names under `<dir>/patches/<name>.json` when `-i` is a directory)
        #[arg(long, value_name = "PATCH", num_args = 1..)]
        patches: Vec<PathBuf>,
    },
    /// Compile a scenario from source files (GeoJSON + JSON) into a single artifact
    Build {
        /// Path to network.geojson
        #[arg(long)]
        network: PathBuf,

        /// Path to demand.json
        #[arg(long)]
        demand: PathBuf,

        /// Path to config.json
        #[arg(long)]
        config: PathBuf,

        /// Path to signals.json (signal groups, plans, tolls; optional)
        #[arg(long)]
        signals: Option<PathBuf>,

        /// Patch JSON files to apply in order
        #[arg(long, value_name = "FILE", num_args = 1..)]
        patches: Vec<PathBuf>,

        /// Output path for scenario.json
        #[arg(short, long)]
        output: PathBuf,
    },
    /// Run a scenario and the built-in verification apparatus on it
    /// (per-element mechanism checks; optionally the analytic benchmarks).
    /// Exit code is non-zero when any check fails, and when the run
    /// exercised nothing at all (absence of evidence is not a pass).
    Verify {
        /// Path to a compiled scenario JSON file or a scenario directory
        #[arg(short, long, value_name = "PATH")]
        input: PathBuf,

        /// Run assignment (generate & route vehicles from demand) before simulation
        #[arg(short, long)]
        assignment: bool,

        /// Aggregation bin size in seconds for conservation / FD checks
        #[arg(long, default_value_t = 300.0)]
        bin_size: f64,

        /// Also run the closed-form analytic benchmarks
        #[arg(long)]
        benchmarks: bool,

        /// Treat any not-exercised mechanism as a failure. Off by default,
        /// because a scenario is entitled not to exercise every mechanism;
        /// on, for gating a suite that is supposed to cover all of them.
        #[arg(long)]
        strict: bool,

        /// Write the full JSON verification report to this path
        #[arg(short, long, value_name = "FILE")]
        output: Option<PathBuf>,

        /// Merge rule at nodes: "in_order" (default, or from config.json in dir mode) or "capacity"
        #[arg(long)]
        merge: Option<String>,

        /// Patch JSON files (or patch names under `<dir>/patches/<name>.json` when `-i` is a directory)
        #[arg(long, value_name = "PATCH", num_args = 1..)]
        patches: Vec<PathBuf>,

        /// Observed field measurements JSON (`observed.json`) for GEH / RMSE comparison
        #[arg(long, value_name = "FILE")]
        observed: Option<PathBuf>,
    },
}

fn merge_rule(name: &str) -> Result<MergeRule> {
    MergeRule::from_config(&serde_json::json!({ "merge": name }))
}

fn read_json(path: &Path) -> Result<serde_json::Value> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("cannot read {}", path.display()))?;
    serde_json::from_str(&text).with_context(|| format!("invalid JSON in {}", path.display()))
}

fn load_patch_file(path: &Path) -> Result<Patch> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("cannot read patch {}", path.display()))?;
    serde_json::from_str(&text).with_context(|| format!("invalid patch JSON in {}", path.display()))
}

fn resolve_patches(patch_args: &[PathBuf], base_dir: Option<&Path>) -> Result<Vec<Patch>> {
    patch_args
        .iter()
        .map(|arg| {
            if arg.exists() {
                return load_patch_file(arg);
            }
            if let Some(dir) = base_dir {
                let candidate = dir.join("patches").join(arg);
                if candidate.exists() {
                    return load_patch_file(&candidate);
                }
                if arg.extension().is_none() {
                    let with_ext = dir.join("patches").join(format!("{}.json", arg.display()));
                    if with_ext.exists() {
                        return load_patch_file(&with_ext);
                    }
                    anyhow::bail!(
                        "patch '{}' not found (checked '{}' and '{}')",
                        arg.display(),
                        candidate.display(),
                        with_ext.display()
                    );
                }
                anyhow::bail!(
                    "patch '{}' not found (checked '{}')",
                    arg.display(),
                    candidate.display()
                );
            }
            load_patch_file(arg)
        })
        .collect()
}

fn load_scenario_for_cli(
    input: &Path,
    patch_args: &[PathBuf],
    merge_flag: Option<&str>,
) -> Result<(Scenario, MergeRule, Vec<String>)> {
    if input.is_dir() {
        let mut inputs = ScenarioInputs::from_dir(input, &[])?;
        inputs.patches = resolve_patches(patch_args, Some(input))?;
        let merge = match merge_flag {
            Some(name) => merge_rule(name)?,
            None => MergeRule::from_config(&inputs.config)?,
        };
        let (scenario, warnings) = compile_inputs(&inputs)?;
        return Ok((scenario, merge, warnings));
    }

    let merge = match merge_flag {
        Some(name) => merge_rule(name)?,
        None => MergeRule::InOrder,
    };
    let mut scenario = load_scenario(input)?;
    let patches = resolve_patches(patch_args, None)?;
    let warnings = apply_patches_to_scenario(&mut scenario, &patches)?;
    scenario.validate()?;
    Ok((scenario, merge, warnings))
}

fn print_patch_warnings(warnings: &[String]) {
    for w in warnings {
        println!("Warning: {}", w);
    }
}

pub fn run() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Build {
            network,
            demand,
            config,
            signals,
            patches,
            output,
        } => {
            println!("Compiling scenario...");
            let start = Instant::now();
            let network_val = read_json(&network)?;
            let demand_val = read_json(&demand)?;
            let config_val = read_json(&config)?;
            let loaded_patches = resolve_patches(&patches, None)?;
            let (mut scenario, warnings) =
                compile_with_patches(network_val, demand_val, config_val, &loaded_patches)?;
            print_patch_warnings(&warnings);

            if let Some(path) = signals {
                let value = read_json(&path)?;
                scenario.signals = stream_core::signals::compile(value, &scenario)?;
            }

            println!("Validating...");
            scenario.validate()?;

            println!("Saving to {:?}...", output);
            use std::io::Write;
            let mut file = std::io::BufWriter::new(std::fs::File::create(output)?);
            serde_json::to_writer_pretty(&mut file, &scenario)?;
            file.flush()?;

            println!("Build complete in {:.2?}.", start.elapsed());
        }
        Commands::Run {
            input,
            output,
            assignment,
            merge,
            bin_size,
            no_vehicles,
            patches,
        } => {
            if !(bin_size.is_finite() && bin_size >= 1.0) {
                anyhow::bail!("bin_size must be a finite number of seconds >= 1");
            }
            println!("Loading scenario from {:?}...", input);
            let start = Instant::now();
            let (mut scenario, merge, warnings) =
                load_scenario_for_cli(&input, &patches, merge.as_deref())?;
            print_patch_warnings(&warnings);
            println!(
                "Loaded in {:.2?}. Vehicles: {}, Demand: {}, Links: {}, Nodes: {}",
                start.elapsed(),
                scenario.vehicles.len(),
                scenario.demand.len(),
                scenario.links.len(),
                scenario.nodes.len()
            );

            if assignment {
                println!("Running Assignment & Generation...");
                let assign_start = Instant::now();

                let added_vehicles = assign_demand(&mut scenario)?;

                println!(
                    "Assignment complete in {:.2?}. Added {} vehicles. Total Vehicles: {}.",
                    assign_start.elapsed(),
                    added_vehicles,
                    scenario.vehicles.len()
                );
            }

            println!("Initializing Simulation...");
            let mut sim = Simulation::new(scenario);
            sim.merge_rule = merge;

            println!("Running Simulation...");
            let sim_start = Instant::now();
            let events_processed = sim.run()?;
            println!(
                "Simulation complete in {:.2?}. Processed {} events.",
                sim_start.elapsed(),
                events_processed
            );

            if let Some(output_path) = output {
                println!("Writing results to {:?}...", output_path);
                let options = RunOptions {
                    assignment,
                    bin_size,
                    keep_vehicles: !no_vehicles,
                    ..Default::default()
                };
                let results = collect(sim.scenario, events_processed, warnings, &options);
                use std::io::Write;
                let mut file = std::io::BufWriter::new(std::fs::File::create(&output_path)?);
                serde_json::to_writer(&mut file, &results)?;
                file.flush()?;
            }
        }
        Commands::Verify {
            input,
            assignment,
            bin_size,
            benchmarks,
            strict,
            output,
            merge,
            patches,
            observed,
        } => {
            run_verify(
                input,
                assignment,
                bin_size,
                benchmarks,
                strict,
                output,
                merge,
                patches,
                observed,
            )?;
        }
    }

    Ok(())
}

#[derive(Serialize)]
struct VerifyWithReference<'a> {
    #[serde(flatten)]
    verification: &'a VerificationReport,
    reference: &'a ReferenceReport,
}

fn load_observed_records(arg: &Path, input: &Path) -> Result<Vec<ObservedRecord>> {
    let path = if arg.exists() {
        arg.to_path_buf()
    } else if input.is_dir() {
        let candidate = input.join(arg);
        if candidate.exists() {
            candidate
        } else {
            anyhow::bail!(
                "observed file '{}' not found (also checked '{}')",
                arg.display(),
                candidate.display()
            );
        }
    } else {
        arg.to_path_buf()
    };
    let text = std::fs::read_to_string(&path)
        .with_context(|| format!("cannot read observed file {}", path.display()))?;
    serde_json::from_str(&text)
        .with_context(|| format!("invalid observed JSON in {}", path.display()))
}

/// Run a scenario through the built-in verification apparatus and print the
/// pass/warn/fail/not-exercised report. Exit contract: Err (non-zero exit)
/// only when a check or benchmark FAILS — warnings and not-exercised do not
/// fail the run, matching the CheckStatus semantics.
#[allow(clippy::too_many_arguments)]
fn run_verify(
    input: PathBuf,
    assignment: bool,
    bin_size: f64,
    benchmarks: bool,
    strict: bool,
    output: Option<PathBuf>,
    merge: Option<String>,
    patches: Vec<PathBuf>,
    observed: Option<PathBuf>,
) -> Result<()> {
    let observed_records = match observed.as_deref() {
        Some(obs) => Some(load_observed_records(obs, &input)?),
        None => None,
    };
    println!("Loading scenario from {:?}...", input);
    let (mut scenario, merge, warnings) =
        load_scenario_for_cli(&input, &patches, merge.as_deref())?;
    print_patch_warnings(&warnings);
    if assignment {
        let added = assign_demand(&mut scenario)?;
        println!("Assignment: {} vehicles generated.", added);
    }
    let mut sim = Simulation::new(scenario);
    sim.merge_rule = merge;
    let events = sim.run()?;
    println!("Simulation: {} events processed.", events);

    let report = verify_scenario(&sim.scenario, bin_size);
    let s = &report.summary;
    println!(
        "Verification: {} elements — {} pass, {} warn, {} fail, {} not-exercised ({} checks, {} failing).",
        s.elements_checked,
        s.elements_pass,
        s.elements_warn,
        s.elements_fail,
        s.elements_not_exercised,
        s.checks_total,
        s.checks_fail
    );
    for element in &report.elements {
        if matches!(element.status, CheckStatus::Fail | CheckStatus::Warn) {
            println!("  [{:?}] {}", element.status, element.label);
            for check in &element.checks {
                if matches!(check.status, CheckStatus::Fail | CheckStatus::Warn) {
                    println!(
                        "      {:?} {}: measured {:?} vs expected {:?} — {}",
                        check.status, check.id, check.measured, check.expected, check.detail
                    );
                }
            }
        }
    }

    let mut benchmark_failures = 0usize;
    if benchmarks {
        println!("Analytic benchmarks:");
        for b in run_benchmarks()? {
            if b.status != CheckStatus::Pass {
                benchmark_failures += 1;
            }
            let ulps = (b.measured.to_bits() as i64 - b.analytic.to_bits() as i64).abs();
            println!(
                "  [{:?}] {} — analytic {} {u}, measured {} {u} ({:.3}% error, {} ulp)",
                b.status,
                b.name,
                b.analytic,
                b.measured,
                b.error_pct,
                ulps,
                u = b.unit
            );
        }
    }

    let reference_report = observed_records.as_deref().map(|recs| {
        let rr = compare_with_observations(&sim.scenario, recs);
        println!(
            "Reference ({} checks): {} pass, {} warn, {} fail.",
            rr.checks.len(),
            rr.n_pass,
            rr.n_warn,
            rr.n_fail
        );
        if let Some(rmse) = rr.flow_rmse_veh_h {
            println!(
                "  Flow: RMSE = {:.1} veh/h, mean GEH = {:.2}",
                rmse,
                rr.flow_mean_geh.unwrap_or(0.0)
            );
        }
        if let Some(rmse) = rr.speed_rmse_kmh {
            println!("  Speed: RMSE = {:.1} km/h", rmse);
        }
        for c in &rr.checks {
            if matches!(c.status, CheckStatus::Fail | CheckStatus::Warn) {
                println!(
                    "  [{:?}] link {} {} [{:.0}, {:.0}): obs {:.1}, sim {:.1} (err {:.1}%{})",
                    c.status,
                    c.link_id,
                    c.label.as_deref().unwrap_or(""),
                    c.t_start,
                    c.t_end,
                    c.observed,
                    c.simulated,
                    c.error_pct,
                    c.geh.map(|g| format!(", GEH {:.2}", g)).unwrap_or_default()
                );
            }
        }
        rr
    });

    if let Some(output_path) = output {
        use std::io::Write;
        let mut file = std::io::BufWriter::new(std::fs::File::create(&output_path)?);
        match &reference_report {
            Some(ref_rep) => {
                let combined = VerifyWithReference {
                    verification: &report,
                    reference: ref_rep,
                };
                serde_json::to_writer_pretty(&mut file, &combined)?;
            }
            None => serde_json::to_writer_pretty(&mut file, &report)?,
        }
        file.flush()?;
        println!("Report written to {:?}.", output_path);
    }

    if s.checks_fail > 0 || benchmark_failures > 0 {
        anyhow::bail!(
            "verification FAILED: {} check(s), {} benchmark(s) failing",
            s.checks_fail,
            benchmark_failures
        );
    }

    let exercised = s.elements_checked.saturating_sub(s.elements_not_exercised);
    if s.elements_checked > 0 && exercised == 0 {
        anyhow::bail!(
            "verification INCONCLUSIVE: no mechanism was exercised ({} element(s), all \
             not-exercised). Nothing was verified. If the scenario draws its vehicles \
             from a demand matrix, pass --assignment.",
            s.elements_checked
        );
    }
    if strict && s.elements_not_exercised > 0 {
        anyhow::bail!(
            "verification INCOMPLETE (--strict): {} of {} element(s) not exercised",
            s.elements_not_exercised,
            s.elements_checked
        );
    }

    if s.elements_not_exercised > 0 {
        println!(
            "Verification passed on what was exercised — {} of {} element(s) \
             not exercised, and therefore not verified.",
            s.elements_not_exercised, s.elements_checked
        );
    } else {
        println!("Verification passed.");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_run_parses_no_vehicles_flag() {
        let default_cli = Cli::try_parse_from(["stream-cli", "run", "-i", "s.json"]).unwrap();
        match default_cli.command {
            Commands::Run {
                no_vehicles,
                bin_size,
                patches,
                merge,
                ..
            } => {
                assert!(!no_vehicles);
                assert_eq!(bin_size, 60.0);
                assert!(patches.is_empty());
                assert!(merge.is_none());
            }
            _ => panic!("expected Run command"),
        }

        let light_cli =
            Cli::try_parse_from(["stream-cli", "run", "-i", "s.json", "--no-vehicles"]).unwrap();
        match light_cli.command {
            Commands::Run { no_vehicles, .. } => assert!(no_vehicles),
            _ => panic!("expected Run command"),
        }
    }

    #[test]
    fn cli_parses_patches_on_build_run_and_verify() {
        let build = Cli::try_parse_from([
            "stream-cli",
            "build",
            "--network",
            "n.geojson",
            "--demand",
            "d.json",
            "--config",
            "c.json",
            "--patches",
            "p1.json",
            "p2.json",
            "-o",
            "out.json",
        ])
        .unwrap();
        match build.command {
            Commands::Build { patches, .. } => {
                assert_eq!(patches, vec![PathBuf::from("p1.json"), PathBuf::from("p2.json")]);
            }
            _ => panic!("expected Build"),
        }

        let run_cmd = Cli::try_parse_from([
            "stream-cli",
            "run",
            "-i",
            "scenarios/a47_givors_osm",
            "--patches",
            "gate",
            "--assignment",
        ])
        .unwrap();
        match run_cmd.command {
            Commands::Run { patches, .. } => assert_eq!(patches, vec![PathBuf::from("gate")]),
            _ => panic!("expected Run"),
        }

        let verify_cmd = Cli::try_parse_from([
            "stream-cli",
            "verify",
            "-i",
            "s.json",
            "--patches",
            "a.json",
            "--patches",
            "b.json",
            "--observed",
            "obs.json",
        ])
        .unwrap();
        match verify_cmd.command {
            Commands::Verify {
                patches, observed, ..
            } => {
                assert_eq!(patches, vec![PathBuf::from("a.json"), PathBuf::from("b.json")]);
                assert_eq!(observed, Some(PathBuf::from("obs.json")));
            }
            _ => panic!("expected Verify"),
        }
    }
}
