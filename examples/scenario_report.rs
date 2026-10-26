//! Simulate a scenario folder, run the verification apparatus and compare
//! with its observations (GEH on flows, relative error on speeds), per
//! station label.
//!
//! cargo run --release --no-default-features --example scenario_report -- scenarios/a47_givors_osm [patch ...]

use std::collections::BTreeMap;
use stream_core_rust::api::{compile, ScenarioInputs};
use stream_core_rust::assignment::assign_demand;
use stream_core_rust::reference::{compare_with_observations, ObservedRecord, ReferenceKind};
use stream_core_rust::simulation::{MergeRule, Simulation};
use stream_core_rust::verification::{verify_scenario, CheckStatus};

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let dir = args.next().unwrap_or_else(|| "scenarios/a47_givors_osm".into());
    let patches: Vec<String> = args.collect();
    let inputs = ScenarioInputs::from_dir(&dir, &patches)?;
    let (mut scenario, warnings) = compile(&inputs)?;
    for w in warnings {
        println!("warning: {w}");
    }
    let generated = assign_demand(&mut scenario)?;
    let mut sim = Simulation::new(scenario);
    sim.merge_rule = MergeRule::from_config(&inputs.config)?;
    // MERGE=in_order|capacity overrides the scenario's config (comparisons).
    if let Ok(m) = std::env::var("MERGE") {
        sim.merge_rule = MergeRule::from_config(&serde_json::json!({ "merge": m }))?;
    }
    println!("merge rule: {:?}", sim.merge_rule);
    let t = std::time::Instant::now();
    let events = sim.run()?;
    let arrived = sim.scenario.vehicles.iter().filter(|v| v.node_times.len() == v.path.len() + 1).count();
    println!("{generated} vehicles, {events} events, {arrived} arrived, {:.2} s", t.elapsed().as_secs_f64());

    let v = verify_scenario(&sim.scenario, 300.0);
    let s = &v.summary;
    println!(
        "verification: {} elements: {} pass, {} warn, {} fail, {} not exercised",
        s.elements_checked, s.elements_pass, s.elements_warn, s.elements_fail, s.elements_not_exercised
    );
    for e in &v.elements {
        if matches!(e.status, CheckStatus::Fail | CheckStatus::Warn) {
            for c in e.checks.iter().filter(|c| matches!(c.status, CheckStatus::Fail | CheckStatus::Warn)) {
                println!("  {:?} {} {}: {}", c.status, e.label, c.id, c.detail);
            }
        }
    }

    let obs_path = std::path::Path::new(&dir).join("observed.json");
    let records: Vec<ObservedRecord> = serde_json::from_str(&std::fs::read_to_string(obs_path)?)?;
    let report = compare_with_observations(&sim.scenario, &records);
    // label -> (flow windows, GEH < 5, mean GEH, speed windows, mean |err| %, sim/obs mean speed)
    let mut by: BTreeMap<String, (usize, usize, f64, usize, f64, f64, f64)> = BTreeMap::new();
    for c in &report.checks {
        let e = by.entry(c.label.clone().unwrap_or_default()).or_default();
        match c.kind {
            ReferenceKind::Flow => {
                e.0 += 1;
                let g = c.geh.unwrap_or(f64::NAN);
                if g < 5.0 {
                    e.1 += 1;
                }
                e.2 += g;
            }
            ReferenceKind::Speed => {
                e.3 += 1;
                e.4 += c.error_pct.abs();
                e.5 += c.simulated;
                e.6 += c.observed;
            }
        }
    }
    println!("\n{:<38} {:>12} {:>8} {:>10} {:>16}", "station", "GEH<5", "GEH", "|dv| %", "v sim/obs km/h");
    let (mut nf, mut np) = (0, 0);
    for (label, (f, p, g, sn, se, ss, so)) in &by {
        nf += f;
        np += p;
        let speed = if *sn > 0 {
            format!("{:>10.0} {:>7.0}/{:<7.0}", se / *sn as f64, ss / *sn as f64, so / *sn as f64)
        } else {
            String::new()
        };
        println!("{label:<38} {:>5}/{:<6} {:>8.2} {speed}", p, f, g / (*f).max(1) as f64);
    }
    println!("\nflow windows with GEH < 5: {np}/{nf} ({:.0} %)", 100.0 * np as f64 / nf.max(1) as f64);

    // REPORT_STATION=<text>: every window of the matching stations.
    if let Ok(pat) = std::env::var("REPORT_STATION") {
        println!("\n{:<34} {:>5}  {:>13}  {:>13}", "station", "time", "q obs/sim", "v obs/sim");
        let mut rows: BTreeMap<(String, i64), (Option<(f64, f64)>, Option<(f64, f64)>)> = BTreeMap::new();
        for c in report.checks.iter().filter(|c| c.label.as_deref().unwrap_or("").contains(&pat)) {
            let e = rows.entry((c.label.clone().unwrap_or_default(), c.t_start as i64)).or_default();
            match c.kind {
                ReferenceKind::Flow => e.0 = Some((c.observed, c.simulated)),
                ReferenceKind::Speed => e.1 = Some((c.observed, c.simulated)),
            }
        }
        for ((label, t), (q, v)) in rows {
            let f = |x: Option<(f64, f64)>| x.map_or("-".to_string(), |(o, s)| format!("{o:.0}/{s:.0}"));
            println!("{label:<34} {:02}:{:02}  {:>13}  {:>13}", t / 3600, t % 3600 / 60, f(q), f(v));
        }
    }
    Ok(())
}
