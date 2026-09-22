use clap::{Parser, Subcommand};
use stream_core_rust::benchmarks::run_benchmarks;
use stream_core_rust::io::{load_scenario, compile_scenario};
use stream_core_rust::simulation::Simulation;
use stream_core_rust::validation::Validate;
use stream_core_rust::assignment::assign_demand;
use stream_core_rust::verification::{verify_scenario, CheckStatus};
use std::path::PathBuf;
use std::time::Instant;
use anyhow::Result;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run a simulation from a compiled scenario JSON
    Run {
        /// Path to the input scenario JSON file
        #[arg(short, long, value_name = "FILE")]
        input: PathBuf,

        /// Path to output results JSON file (optional)
        #[arg(short, long, value_name = "FILE")]
        output: Option<PathBuf>,

        /// Run assignment (generate & route vehicles from demand) before simulation
        #[arg(short, long)]
        assignment: bool,
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
        
        /// Output path for scenario.json
        #[arg(short, long)]
        output: PathBuf,
    },
    /// Run a scenario and the built-in verification apparatus on it
    /// (per-element mechanism checks; optionally the analytic benchmarks).
    /// Exit code is non-zero when any check fails.
    Verify {
        /// Path to the input scenario JSON file (compiled artifact)
        #[arg(short, long, value_name = "FILE")]
        input: PathBuf,

        /// Run assignment (generate & route vehicles from demand) before simulation
        #[arg(short, long)]
        assignment: bool,

        /// Aggregation bin size in seconds for conservation / FD checks
        #[arg(long, default_value_t = 300.0)]
        bin_size: f64,

        /// Also run the nine closed-form analytic benchmarks
        #[arg(long)]
        benchmarks: bool,

        /// Write the full JSON verification report to this path
        #[arg(short, long, value_name = "FILE")]
        output: Option<PathBuf>,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Build { network, demand, config, output } => {
            println!("Compiling scenario...");
            let start = Instant::now();
            let scenario = compile_scenario(network, demand, config)?;
            
            println!("Validating...");
            scenario.validate()?;
            
            println!("Saving to {:?}...", output);
            use std::io::Write;
            let mut file = std::io::BufWriter::new(std::fs::File::create(output)?);
            serde_json::to_writer_pretty(&mut file, &scenario)?;
            file.flush()?;
            
            println!("Build complete in {:.2?}.", start.elapsed());
        },
        Commands::Run { input, output, assignment } => {
            println!("Loading scenario from {:?}...", input);
            let start = Instant::now();
            let mut scenario = load_scenario(&input)?;
            println!("Loaded in {:.2?}. Vehicles: {}, Demand: {}, Links: {}, Nodes: {}", 
                start.elapsed(), scenario.vehicles.len(), scenario.demand.len(), scenario.links.len(), scenario.nodes.len());

            println!("Validating scenario...");
            scenario.validate()?;
            println!("Validation passed.");

            if assignment {
                println!("Running Assignment & Generation...");
                let assign_start = Instant::now();
                
                let added_vehicles = assign_demand(&mut scenario)?;
                
                println!("Assignment complete in {:.2?}. Added {} vehicles. Total Vehicles: {}.", 
                    assign_start.elapsed(), added_vehicles, scenario.vehicles.len());
            }

            println!("Initializing Simulation...");
            let mut sim = Simulation::new(scenario);

            println!("Running Simulation...");
            let sim_start = Instant::now();
            let events_processed = sim.run()?;
            println!("Simulation complete in {:.2?}. Processed {} events.", sim_start.elapsed(), events_processed);

            if let Some(output_path) = output {
                println!("Writing results to {:?}...", output_path);
                // TODO: Result export
            }
        }
        Commands::Verify { input, assignment, bin_size, benchmarks, output } => {
            run_verify(input, assignment, bin_size, benchmarks, output)?;
        }
    }

    Ok(())
}

/// Run a scenario through the built-in verification apparatus and print the
/// pass/warn/fail/not-exercised report. Exit contract: Err (non-zero exit)
/// only when a check or benchmark FAILS — warnings and not-exercised do not
/// fail the run, matching the CheckStatus semantics.
fn run_verify(
    input: PathBuf,
    assignment: bool,
    bin_size: f64,
    benchmarks: bool,
    output: Option<PathBuf>,
) -> Result<()> {
    println!("Loading scenario from {:?}...", input);
    let mut scenario = load_scenario(&input)?;
    scenario.validate()?;
    if assignment {
        let added = assign_demand(&mut scenario)?;
        println!("Assignment: {} vehicles generated.", added);
    }
    let mut sim = Simulation::new(scenario);
    let events = sim.run()?;
    println!("Simulation: {} events processed.", events);

    let report = verify_scenario(&sim.scenario, bin_size);
    let s = &report.summary;
    println!(
        "Verification: {} elements — {} pass, {} warn, {} fail, {} not-exercised ({} checks, {} failing).",
        s.elements_checked, s.elements_pass, s.elements_warn, s.elements_fail,
        s.elements_not_exercised, s.checks_total, s.checks_fail
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
            if b.status == CheckStatus::Fail {
                benchmark_failures += 1;
            }
            println!(
                "  [{:?}] {} — analytic {} {u}, measured {} {u} ({:.3}% error)",
                b.status, b.name, b.analytic, b.measured, b.error_pct, u = b.unit
            );
        }
    }

    if let Some(output_path) = output {
        use std::io::Write;
        let mut file = std::io::BufWriter::new(std::fs::File::create(&output_path)?);
        serde_json::to_writer_pretty(&mut file, &report)?;
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
    println!("Verification passed.");
    Ok(())
}
