# 1. Quickstart

`stream-cli` is the standalone command-line executable for compiling, simulating, and verifying traffic scenarios.

## Installing `stream-cli`

Download a pre-built archive for Linux, Windows, or macOS from the [GitHub Releases](https://github.com/AureClai/stream-pipes-rust/releases) page and place `stream-cli` (`stream-cli.exe` on Windows) on your `PATH`.

### Building from source

Requires a stable Rust toolchain (edition 2021, Rust >= 1.82).

```bash
cargo build --release --bin stream-cli
```

On Windows, the compiled executable is at:

```text
target\release\stream-cli.exe
```

## Typical Workflow: `build` → `verify` → `run`

### 1. Compile source files into a scenario

Combine `network.geojson`, `demand.json`, and `config.json` into a single validated `scenario.json`:

```bash
stream-cli build \
  --network scenarios/diverge/network.geojson \
  --demand scenarios/diverge/demand.json \
  --config scenarios/diverge/config.json \
  -o scenario.json
```

### 2. Verify physics & conservation

Run the simulation together with the built-in LWR verification checks (and optionally the 18 closed-form analytic benchmarks):

```bash
stream-cli verify -i scenario.json --assignment --benchmarks -o report.json
```

`verify` exits with a non-zero code if any check or benchmark fails, making it suitable for CI gates.

### 3. Run and export results

Run the simulation and write binned link/pipe statistics to JSON:

```bash
stream-cli run -i scenario.json --assignment --bin-size 60 -o results.json
```

- Omit `--assignment` if `scenario.json` already contains pre-routed `vehicles`.
- `--bin-size` sets the aggregation window in seconds (default: `60.0`).

See [JSON Format Reference](09-json-format.md) for the exact structure of every file.
