"""
monte_carlo_final.py
====================
Complete Monte Carlo analysis for AEB lateral intrusion gap study.

Author:  Tejas Manjunath
Project: Lateral Intrusion ADAS Gap — AEB Blind Zone Study

Description:
  Runs 100,000 randomised lateral departure scenarios to quantify the
  collision risk under production ADAS sensor architectures. Evaluates
  baseline production sensor configuration and performs counterfactual
  comparison across four sensor configurations. Includes multi-seed
  validation, sensitivity analysis, and actuation parameter sweep.

Validated results:
  Baseline collision rate:  99.6%
  Corner radar improvement: −22.1 pp (99.6% → 77.4%)
  Physics-limited floor:    46.6% (perfect detection)
  Multi-seed std:           0.020% (seeds 42, 123, 999)

Parameter ranges (Monte Carlo inputs):
  Speed:             60–100 km/h (stratified, 60% highway band)
  Lateral drift:     0.3–1.2 m/s + Gaussian noise ±10%
  Lane gap:          2.5–5.0 m + Gaussian noise ±5%
  Longitudinal offset: ±10 m (true lateral geometry only)

Sensor architecture modelled:
  Forward radar:  35° H-FOV, 150m range (FOV gap — lateral targets
                  always outside forward detection cone)
  Side radar:     120° H-FOV, 8m range (parking-assist spec,
                  non-actionable above 10 km/h)
  Rear radar:     150° H-FOV, 80m range (geometry gap — intruder
                  ahead/beside, not behind)
  Corner radar:   120° H-FOV, 50m range (counterfactual only)

S3 calibration (CARLA 0.9.16 baseline):
  CARLA scenario:  3.5m gap, 0.7 m/s drift, 80 km/h
  Measured event duration: 4.15s (physics-based collision)
  Mathematical prediction: 5.0s (gap / drift)
  Correction factor: 0.83 (ratio of measured to predicted)
  Application: applied globally to all time-to-cross calculations

Outputs (saved to mc_counterfactual_output/):
  cf_comparison.png       — four-bar architecture comparison chart
  cf_summary.csv          — collision rates per configuration
  actuation_heatmap.png   — reaction time × deceleration sweep
  actuation_sweep.csv     — raw actuation sweep data

Requirements:
  Python 3.10+
  numpy, matplotlib, pandas
  pip install numpy matplotlib pandas

Usage:
  python monte_carlo_final.py

Note on parameter scope:
  Longitudinal offset is restricted to ±10m to model true lateral
  departure geometry (intruding vehicle beside ego). Wider ranges
  (−30 to +50m) include cut-in scenarios that inflate AEB viability
  and are excluded from this analysis by design.
"""

# ── IMPORTS ────────────────────────────────────────────────────────────────
import os
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for server environments
import matplotlib.pyplot as plt


# ── CONFIGURATION ──────────────────────────────────────────────────────────
OUTPUT_DIR = 'mc_counterfactual_output'
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── MONTE CARLO PARAMETERS ─────────────────────────────────────────────────
N_SCENARIOS = 100_000  # Number of randomised scenarios per simulation run
SEED = 42  # Primary random seed for reproducibility
MULTI_SEEDS = [42, 123, 999]  # Seeds for multi-run validation


# ── SENSOR SPECIFICATIONS ──────────────────────────────────────────────────
# Side radar parking-assist spec (production baseline)
SIDE_RADAR_RANGE = 5.0  # meters — detection range
SIDE_RADAR_MAX_SPEED = 10.0  # km/h — actionable speed threshold (parking maneuvers only)

# Side radar highway-rated spec (counterfactual improved)
IMPROVED_RADAR_SPEED = 80.0  # km/h — extended actionable speed for highway use

# Forward radar AEB spec
FWD_RADAR_FOV = 35.0  # degrees — horizontal field of view (creates lateral blind zone)

# Corner radar spec (counterfactual addition)
CORNER_FOV = 120.0  # degrees — wide horizontal field of view
CORNER_RANGE = 50.0  # meters — medium-range detection


# ── AEB SYSTEM PARAMETERS ──────────────────────────────────────────────────
MIN_AEB_RESPONSE = 1.5  # seconds — minimum time window required to trigger AEB
REACTION_TIME = 1.2  # seconds — total system delay (sensor processing + ECU + actuator)
DECELERATION = 8.0  # m/s² — maximum braking deceleration (realistic production limit)


# ── S3 CARLA BASELINE (validation reference) ───────────────────────────────
# Correction factor derived from CARLA S3 baseline scenario
# CARLA measured: 4.15s event duration (3.5m gap / 0.7 m/s drift with physics)
# Mathematical prediction: 5.0s (gap / drift, no physics correction)
# Ratio: 4.15 / 5.0 = 0.83
S3_CORRECTION = 0.83  # Applied globally to time-to-cross calculations


# ── PARAMETER RANGES ───────────────────────────────────────────────────────
PARAMS = {
    'lateral_drift': {'min': 0.3, 'max': 1.2},  # m/s — intruding vehicle lateral speed
    'lane_gap': {'min': 2.5, 'max': 5.0},  # meters — initial lateral separation
    'speed': {'min': 60, 'max': 100},  # km/h — both vehicles travel at same speed
    'long_offset': {'min': -10.0, 'max': 10.0},  # meters — true lateral geometry only
}


# ── VALIDATION TARGETS ─────────────────────────────────────────────────────
CONFIRMED_BASELINE = 99.6  # percent — validated collision rate (true lateral geometry, 3-seed mean)
TOLERANCE = 0.5  # percent — acceptable deviation from confirmed baseline


# ── MONTE CARLO SIMULATION ─────────────────────────────────────────────────
def run_monte_carlo(mode='baseline', seed=SEED, n=N_SCENARIOS):
    """
    Execute Monte Carlo simulation across randomised lateral intrusion scenarios.
    
    Args:
        mode: Detection configuration ('baseline', 'improved', 'corner', 'perfect')
        seed: Random seed for reproducibility
        n: Number of scenarios to simulate
    
    Returns:
        List of scenario result dictionaries containing kinematic parameters,
        detection status, AEB viability, and collision outcome
    
    Physics model (identical across all modes):
        time_to_respond = reaction_time + (speed / deceleration)
        aeb_viable = time_to_cross > time_to_respond (if detected)
        collision = not aeb_viable
    
    Detection layer (varies by mode):
        baseline  — side <10 km/h + forward + rear (production spec)
        improved  — side <80 km/h + forward + rear
        corner    — improved + corner radar (120°, 50m)
        perfect   — any_detection = True (physics floor)
    """
    rng = np.random.default_rng(seed)
    results = []

    for _ in range(n):

        # ── PARAMETER SAMPLING ─────────────────────────────────────────────
        # Draw base parameters from uniform distributions
        lat_drift = rng.uniform(PARAMS['lateral_drift']['min'],
                                PARAMS['lateral_drift']['max'])
        lane_gap = rng.uniform(PARAMS['lane_gap']['min'],
                               PARAMS['lane_gap']['max'])
        long_offset = rng.uniform(PARAMS['long_offset']['min'],
                                  PARAMS['long_offset']['max'])
        
        # Stratified speed sampling — 60% highway (80-100 km/h), 40% lower
        speed_kmh = (rng.uniform(80, 100) if rng.random() < 0.6
                     else rng.uniform(60, 80))

        # ── NOISE APPLICATION ──────────────────────────────────────────────
        # Apply Gaussian noise to lateral drift and lane gap
        lat_drift *= rng.normal(1.0, 0.1)  # ±10% variation
        lane_gap *= rng.normal(1.0, 0.05)  # ±5% variation
        lat_drift = max(0.05, lat_drift)  # Enforce minimum drift
        lane_gap = max(0.5, lane_gap)  # Enforce minimum gap

        # Convert speed and calculate time-to-cross with S3 correction
        speed_mps = speed_kmh / 3.6
        # S3 correction factor accounts for CARLA physics discrepancy
        time_to_cross = (lane_gap / lat_drift) * S3_CORRECTION

        # ── FORWARD RADAR ANALYSIS ─────────────────────────────────────────
        # Calculate detection angle — intruder must be within forward cone
        angle = np.degrees(np.arctan2(lane_gap, abs(long_offset) + 1e-6))
        in_fov = (angle <= FWD_RADAR_FOV / 2) and (long_offset > 0)
        fwd_window = (lane_gap / lat_drift) if in_fov else 0.0
        fwd_actionable = in_fov and (fwd_window >= MIN_AEB_RESPONSE)

        # ── SIDE RADAR ANALYSIS ────────────────────────────────────────────
        # Detection starts when intruder enters side radar range
        side_detect_start = ((lane_gap - SIDE_RADAR_RANGE) / lat_drift
                             if lane_gap > SIDE_RADAR_RANGE else 0.0)
        side_raw_window = max(0.0, time_to_cross - side_detect_start)

        # Trigger probability model — function of distance and speed
        dist_factor = max(0.0, min(1.0,
                                   (SIDE_RADAR_RANGE - lane_gap) / SIDE_RADAR_RANGE))
        speed_factor = max(0.0, min(1.0, 1.0 - speed_kmh / 120.0))
        trigger_prob = min(0.8, 0.6 * dist_factor + 0.4 * speed_factor)

        # Baseline spec: parking-assist (<10 km/h actionable speed)
        side_actionable = (
            side_raw_window >= MIN_AEB_RESPONSE and
            speed_kmh < SIDE_RADAR_MAX_SPEED and
            rng.random() < trigger_prob and
            abs(long_offset) < 3.0
        )
        
        # Improved spec: highway-rated (<80 km/h actionable speed)
        side_actionable_hw = (
            side_raw_window >= MIN_AEB_RESPONSE and
            speed_kmh < IMPROVED_RADAR_SPEED and
            rng.random() < trigger_prob and
            abs(long_offset) < 3.0
        )

        # ── REAR RADAR ANALYSIS ────────────────────────────────────────────
        # Only detects when intruder is behind ego (long_offset < 0)
        if long_offset < 0:
            rear_window = abs(long_offset) / speed_mps
            rear_actionable = rear_window >= MIN_AEB_RESPONSE
        else:
            rear_window = 0.0
            rear_actionable = False

        # ── CORNER RADAR ANALYSIS ──────────────────────────────────────────
        # Wide FOV corner-mounted radar (counterfactual configuration)
        corner_angle = np.degrees(np.arctan2(lane_gap, long_offset + 1e-6))
        corner_detected = (abs(corner_angle) <= CORNER_FOV / 2 and
                           0 < long_offset < CORNER_RANGE)
        corner_actionable = corner_detected and time_to_cross >= MIN_AEB_RESPONSE

        # ── DETECTION LAYER ────────────────────────────────────────────────
        # Only difference between modes — which sensors are active
        if mode == 'baseline':
            any_detection = side_actionable or fwd_actionable or rear_actionable

        elif mode == 'improved':
            any_detection = side_actionable_hw or fwd_actionable or rear_actionable

        elif mode == 'corner':
            any_detection = (side_actionable_hw or fwd_actionable or
                             rear_actionable or corner_actionable)

        else:  # perfect
            any_detection = True

        # ── PHYSICS MODEL ──────────────────────────────────────────────────
        # Identical physics for all modes — single source of truth
        # Time-to-respond: full reaction time + braking time to stop from current speed
        # AEB viable only if crossing time exceeds total response time
        if any_detection:
            time_to_respond = REACTION_TIME + (speed_mps / DECELERATION)
            aeb_viable = time_to_cross > time_to_respond
        else:
            aeb_viable = False

        # Collision determination and impact speed calculation
        collision = not aeb_viable
        impact_speed = speed_mps if collision else 0.0

        # ── RESULTS STORAGE ────────────────────────────────────────────────
        results.append({
            'lat_drift': round(lat_drift, 3),
            'lane_gap': round(lane_gap, 3),
            'speed_kmh': round(speed_kmh, 1),
            'long_offset': round(long_offset, 2),
            'time_to_cross': round(time_to_cross, 3),
            'side_raw_window': round(side_raw_window, 3),
            'any_detection': any_detection,
            'aeb_viable': aeb_viable,
            'collision': collision,
            'impact_speed': round(impact_speed, 3),
            'impact_severity': ('high' if impact_speed > 20 else
                                'medium' if impact_speed > 10 else 'low'),
        })

    return results


def collision_rate(results):
    """
    Calculate collision rate as percentage of total scenarios.
    
    Args:
        results: List of scenario result dictionaries
    
    Returns:
        Collision rate as percentage (0-100)
    """
    return 100.0 * sum(r['collision'] for r in results) / len(results)


def summarise(results):
    """
    Generate summary statistics from simulation results.
    
    Args:
        results: List of scenario result dictionaries
    
    Returns:
        Dictionary containing collision rate, AEB viability rate,
        and mean impact speed for collisions
    """
    cr = collision_rate(results)
    imp = [r['impact_speed'] for r in results if r['collision']]
    return {
        'collision_rate': round(cr, 2),
        'aeb_viable_rate': round(100 - cr, 2),
        'mean_impact_ms': round(float(np.mean(imp)), 2) if imp else 0.0,
    }


# ── MULTI-SEED VALIDATION ──────────────────────────────────────────────────
def multi_seed_validation():
    """
    Validate baseline collision rate across multiple random seeds.
    
    Runs baseline simulation with three different seeds to verify
    result stability and validate against confirmed baseline.
    
    Returns:
        Tuple of (mean_rate, std_rate, validation_passed)
    """
    print("\n── MULTI-SEED BASELINE VALIDATION ──")
    rates = []
    for seed in MULTI_SEEDS:
        r = collision_rate(run_monte_carlo('baseline', seed))
        rates.append(r)
        print(f"  Seed {seed:>4}: {r:.2f}%")
    mean, std = float(np.mean(rates)), float(np.std(rates))
    print(f"  Mean: {mean:.2f}%   Std: {std:.3f}%")
    ok = abs(mean - CONFIRMED_BASELINE) <= TOLERANCE
    print(f"  Target {CONFIRMED_BASELINE}% ±{TOLERANCE}%: {'PASS ✓' if ok else 'OUTSIDE — review'}")
    return mean, std, ok


# ── SENSITIVITY ANALYSIS ───────────────────────────────────────────────────
def sensitivity_analysis(results):
    """
    Analyse collision rate sensitivity across parameter bands.
    
    Args:
        results: Baseline simulation results
    
    Prints collision rates stratified by speed, lateral drift, and lane gap
    to identify parameter sensitivities in the baseline configuration.
    """
    print("\n── SENSITIVITY ANALYSIS (baseline) ──")
    
    def rate(s):
        """Calculate collision rate for scenario subset."""
        return 0.0 if not s else 100 * sum(r['collision'] for r in s) / len(s)
    
    # Speed band sensitivity
    print("  Speed band:")
    for lo, hi in [(60, 70), (70, 80), (80, 90), (90, 100)]:
        s = [r for r in results if lo <= r['speed_kmh'] <= hi]
        print(f"    {lo}–{hi} km/h → {rate(s):.2f}%  (n={len(s)})")
    
    # Lateral drift sensitivity
    print("  Lateral drift:")
    for lo, hi in [(0.3, 0.6), (0.6, 0.9), (0.9, 1.2)]:
        s = [r for r in results if lo <= r['lat_drift'] <= hi]
        print(f"    {lo}–{hi} m/s  → {rate(s):.2f}%  (n={len(s)})")
    
    # Lane gap sensitivity
    print("  Lane gap:")
    for lo, hi in [(2.5, 3.5), (3.5, 4.5), (4.5, 5.0)]:
        s = [r for r in results if lo <= r['lane_gap'] <= hi]
        print(f"    {lo}–{hi} m    → {rate(s):.2f}%  (n={len(s)})")


# ── ACTUATION SWEEP ────────────────────────────────────────────────────────
def actuation_sweep():
    """
    Sweep reaction time and deceleration parameters under perfect detection.
    
    Tests 12 combinations of reaction time (1.2s, 1.0s, 0.8s, 0.5s) and
    deceleration (8.0, 9.0, 10.0 m/s²) to quantify physics-limited floor
    and identify viable actuation parameter ranges.
    
    Outputs:
        actuation_sweep.csv — raw collision rates per parameter combination
        actuation_heatmap.png — visual heatmap of actuation parameter space
    """
    print("\n── ACTUATION SWEEP (perfect detection) ──")
    rts = [1.2, 1.0, 0.8, 0.5]  # Reaction time values (seconds)
    decs = [8.0, 9.0, 10.0]  # Deceleration values (m/s²)
    rows = []
    print(f"  {'Reaction':>10}  {'Decel':>8}  {'Collision':>10}")

    for rt in rts:
        for dec in decs:
            rng = np.random.default_rng(SEED)
            cols = 0
            for _ in range(N_SCENARIOS):
                # Identical parameter sampling to main simulation
                lat_drift = rng.uniform(0.3, 1.2)
                lane_gap = rng.uniform(2.5, 5.0)
                rng.uniform(-30.0, 50.0)  # Consume long_offset draw to preserve RNG state
                speed_kmh = (rng.uniform(80, 100) if rng.random() < 0.6
                             else rng.uniform(60, 80))
                lat_drift *= rng.normal(1.0, 0.1)
                lane_gap *= rng.normal(1.0, 0.05)
                lat_drift = max(0.05, lat_drift)
                lane_gap = max(0.5, lane_gap)
                speed_mps = speed_kmh / 3.6
                time_to_cross = (lane_gap / lat_drift) * S3_CORRECTION
                
                # Perfect detection — physics with custom reaction time and deceleration
                time_to_respond = rt + (speed_mps / dec)
                if not (time_to_cross > time_to_respond):
                    cols += 1
                    
            rate = round(100 * cols / N_SCENARIOS, 2)
            rows.append({'reaction_time': rt, 'deceleration': dec,
                         'collision_rate': rate})
            print(f"  {rt:>9}s  {dec:>6.1f} m/s²  {rate:>9.1f}%")

    # Save CSV output
    csv_path = os.path.join(OUTPUT_DIR, 'actuation_sweep.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)

    # Generate heatmap visualisation
    rt_vals = sorted(set(r['reaction_time'] for r in rows))
    dec_vals = sorted(set(r['deceleration'] for r in rows), reverse=True)
    lookup = {(r['reaction_time'], r['deceleration']): r['collision_rate']
              for r in rows}
    mat = np.array([[lookup[(rt, dec)] for rt in rt_vals]
                     for dec in dec_vals])

    fig, ax = plt.subplots(figsize=(9, 5))
    fig.patch.set_facecolor('white')
    im = ax.imshow(mat, cmap='RdYlGn_r', aspect='auto',
                   vmin=float(mat.min()) - 2, vmax=100)
    plt.colorbar(im, ax=ax, label='Collision Rate (%)')
    ax.set_xticks(range(len(rt_vals)))
    ax.set_xticklabels([f'{v}s' for v in rt_vals], fontsize=11)
    ax.set_yticks(range(len(dec_vals)))
    ax.set_yticklabels([f'{v} m/s²' for v in dec_vals], fontsize=11)
    ax.set_xlabel('Reaction Time (s)', fontsize=12)
    ax.set_ylabel('Deceleration (m/s²)', fontsize=12)
    ax.set_title('Actuation Sweep — Collision Rate Under Perfect Detection\n'
                 f'100,000 scenarios per cell  ·  Single physics model',
                 fontweight='bold')
    
    # Add cell annotations
    for r_i in range(len(dec_vals)):
        for c_i in range(len(rt_vals)):
            v = mat[r_i, c_i]
            ax.text(c_i, r_i, f'{v:.1f}%', ha='center', va='center',
                    fontsize=12, fontweight='bold',
                    color='white' if v > 80 else 'black')
    
    plt.tight_layout()
    hm = os.path.join(OUTPUT_DIR, 'actuation_heatmap.png')
    plt.savefig(hm, dpi=150, facecolor='white', bbox_inches='tight')
    plt.close()
    print(f"  Heatmap: {hm}")
    return rows


# ── OUTPUT / VISUALISATION ─────────────────────────────────────────────────
def save_comparison_chart(rates):
    """
    Generate counterfactual comparison bar chart.
    
    Args:
        rates: Dictionary mapping mode names to collision rates
    
    Outputs:
        cf_comparison.png — four-bar chart comparing sensor configurations
    """
    modes = ['baseline', 'improved', 'corner', 'perfect']
    labels = ['Baseline\n(side radar <10 km/h)',
              'Improved\n(highway-rated radar)',
              'Corner Radar\n(120° FOV, 50 m)',
              'Perfect detection\n(physics floor)']
    colors = ['#c42c2c', '#d4711a', '#2471a3', '#1e8449']
    values = [rates[m] for m in modes]
    base = values[0]

    fig, ax = plt.subplots(figsize=(12, 7))
    fig.patch.set_facecolor('white')
    bars = ax.bar(labels, values, color=colors, alpha=0.88, width=0.5, zorder=3)
    
    # Add value labels on bars
    for bar, v in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.4,
                f'{v:.1f}%', ha='center', fontsize=13, fontweight='bold')
    
    # Add baseline reference line
    ax.axhline(base, color='#c42c2c', linestyle='--', linewidth=1.2,
               alpha=0.35, label=f'Baseline {base:.1f}%')
    
    ax.set_ylabel('Collision Rate (%)', fontsize=12)
    ax.set_ylim(0, 105)
    ax.set_title(
        'Counterfactual Sensor Architecture Comparison\n'
        f'100,000 scenarios  ·  Single time-based physics model  ·  Seed={SEED}',
        fontsize=13, fontweight='bold')
    ax.grid(axis='y', alpha=0.3, zorder=1)
    ax.legend(fontsize=10)
    
    # Add summary note box
    note = '\n'.join([
        f'Improved  vs Baseline: −{base - values[1]:.1f} pp',
        f'Corner    vs Baseline: −{base - values[2]:.1f} pp',
        f'Perfect   vs Baseline: −{base - values[3]:.1f} pp (physics floor)',
        f'Sensing-driven gap:    {base - values[3]:.1f} pp of {base:.1f}%',
    ])
    ax.text(0.98, 0.97, note, transform=ax.transAxes, ha='right', va='top',
            fontsize=10,
            bbox=dict(boxstyle='round', facecolor='#f5f7fa',
                      edgecolor='#b0b8c4', alpha=0.9))
    
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'cf_comparison.png')
    plt.savefig(path, dpi=150, facecolor='white', bbox_inches='tight')
    plt.close()
    print(f"  Chart: {path}")


# ── MAIN ───────────────────────────────────────────────────────────────────
def main():
    """
    Execute complete Monte Carlo analysis pipeline.
    
    Pipeline:
      1. Multi-seed baseline validation
      2. Four-mode counterfactual comparison
      3. Sensitivity analysis on baseline
      4. Actuation parameter sweep
      5. Output generation (charts and CSV)
    """
    print("=" * 65)
    print("COUNTERFACTUAL AEB ANALYSIS — single model, one physics block")
    print(f"N={N_SCENARIOS:,}  ·  Seed={SEED}  ·  S3_CORRECTION={S3_CORRECTION}")
    print(f"Physics: time_to_cross > reaction + braking_time")
    print("=" * 65)

    # Validate baseline against confirmed result
    mean_cr, std_cr, ok = multi_seed_validation()
    if not ok:
        print(f"\nWARNING: baseline {mean_cr:.2f}% outside target. "
              "Check detection logic before publishing.")

    # Run full counterfactual comparison
    print(f"\n── FULL SIMULATION (seed={SEED}) ──")
    modes = ['baseline', 'improved', 'corner', 'perfect']
    results = {}
    rates = {}
    for mode in modes:
        print(f"  Running {mode}...", end=' ', flush=True)
        results[mode] = run_monte_carlo(mode)
        s = summarise(results[mode])
        rates[mode] = s['collision_rate']
        print(f"collision={s['collision_rate']}%  "
              f"aeb_viable={s['aeb_viable_rate']}%  "
              f"mean_impact={s['mean_impact_ms']} m/s")

    # Run sensitivity analysis on baseline
    sensitivity_analysis(results['baseline'])

    # Print counterfactual comparison summary
    base = rates['baseline']
    print(f"\n{'=' * 65}")
    print("COUNTERFACTUAL RESULTS")
    print(f"{'=' * 65}")
    print(f"  {'Mode':<32} {'Collision':>10}  {'vs Baseline':>14}")
    print(f"  {'-' * 32} {'-' * 10}  {'-' * 14}")
    for mode in modes:
        cr = rates[mode]
        tag = '—' if mode == 'baseline' else f'−{base - cr:.1f} pp'
        print(f"  {mode:<32} {cr:>9.1f}%  {tag:>14}")
    print(f"\n  Sensing-driven failures: {base - rates['perfect']:.1f} pp of {base:.1f}%")
    print(f"  Physics-limited floor:  {rates['perfect']:.1f}%")
    print("=" * 65)

    # Save outputs
    print("\n── SAVING OUTPUTS ──")
    save_comparison_chart(rates)
    csv_path = os.path.join(OUTPUT_DIR, 'cf_summary.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['mode', 'collision_rate',
                                          'reduction_vs_baseline'])
        w.writeheader()
        for mode in modes:
            w.writerow({'mode': mode, 'collision_rate': rates[mode],
                        'reduction_vs_baseline': round(base - rates[mode], 2)})
    print(f"  CSV: {csv_path}")

    # Run actuation sweep
    actuation_sweep()
    print(f"\nAll outputs: {OUTPUT_DIR}/")


if __name__ == '__main__':
    main()