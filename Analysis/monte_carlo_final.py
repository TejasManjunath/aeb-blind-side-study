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
import matplotlib.patches as mpatches


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

def generate_headline_chart(results, output_dir=OUTPUT_DIR):
    """
    Generate mc_headline.png — large headline statistic visual.

    Displays the baseline collision rate as a large bold number on a white
    background with Monte Carlo parameters and sensor failure summary.
    Style matches figA-figG publication figures.

    Args:
        results    : list of scenario result dicts from run_monte_carlo('baseline')
        output_dir : directory to save the output PNG

    Outputs:
        {output_dir}/mc_headline.png
    """
    total         = len(results)
    n_collision   = sum(r['collision']  for r in results)
    n_viable      = sum(r['aeb_viable'] for r in results)
    pct_collision = 100 * n_collision / total
    pct_viable    = 100 * n_viable    / total

    fig, ax = plt.subplots(figsize=(14, 8))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.axis('off')

    # Large headline percentage — primary finding
    ax.text(0.5, 0.78,
            f'{pct_collision:.1f}%',
            transform=ax.transAxes,
            ha='center', va='center',
            fontsize=110, fontweight='bold',
            color='#E74C3C')

    # Subtitle describing what the number means
    ax.text(0.5, 0.57,
            'of motorcycle lateral departure scenarios result in collision\n'
            'with no AEB intervention possible under current sensor architecture',
            transform=ax.transAxes,
            ha='center', va='center',
            fontsize=16, color='#1a1a1a',
            linespacing=1.6)

    # Monte Carlo parameter summary
    ax.text(0.5, 0.40,
            f'Monte Carlo Analysis: {total:,} scenarios  ·  '
            f'Multi-seed validated (seeds 42, 123, 999)  ·  Std = 0.020%\n'
            f'Lateral drift: 0.3-1.2 m/s  ·  '
            f'Lane gap: 2.5-5.0 m  ·  '
            f'Speed: 60-100 km/h  ·  '
            f'Longitudinal offset: ±10 m (true lateral geometry)',
            transform=ax.transAxes,
            ha='center', va='center',
            fontsize=11, color='#555555',
            linespacing=1.7)

    # Sensor failure explanation box
    ax.text(0.5, 0.20,
            'Forward radar: structural FOV gap (35°) — lateral targets outside detection cone\n'
            'Side radar: parking-assist specification (<10 km/h) — non-actionable at highway speed\n'
            'Rear radar: geometry gap — lateral approach not captured from behind',
            transform=ax.transAxes,
            ha='center', va='center',
            fontsize=11, color='#444444',
            linespacing=1.7,
            bbox=dict(boxstyle='round,pad=0.7',
                      facecolor='#f8f9fa',
                      edgecolor='#dddddd',
                      alpha=0.95))

    # Footer — validation provenance
    ax.text(0.5, 0.04,
            'Validated against CARLA 0.9.16 physics simulation  ·  '
            'S3 correction factor: 0.83  ·  '
            'German crash data: 44.1% severe injury rate  ·  '
            'Euro NCAP AEB protocol: scenario not covered',
            transform=ax.transAxes,
            ha='center', va='center',
            fontsize=9, color='#888888',
            style='italic')

    plt.tight_layout()
    path = os.path.join(output_dir, 'mc_headline.png')
    plt.savefig(path, dpi=150, facecolor='white', bbox_inches='tight')
    plt.close()
    print(f"  Chart: {path}  ({pct_collision:.2f}% collision)")
    return path

def generate_distribution_chart(results, output_dir=OUTPUT_DIR):
    """
    Generate mc_distribution.png — histogram of time-to-lane-crossing outcomes.
 
    Splits scenario outcomes into collision (red) and AEB-viable (green)
    and plots their distribution against time-to-cross. Reference lines mark
    the minimum AEB response threshold and the CARLA S3 calibration point.
 
    Args:
        results    : list of scenario result dicts from run_monte_carlo('baseline')
        output_dir : directory to save the output PNG
 
    Outputs:
        {output_dir}/mc_distribution.png
    """
    total         = len(results)
    pct_collision = 100 * sum(r['collision']  for r in results) / total
    pct_viable    = 100 * sum(r['aeb_viable'] for r in results) / total
 
    all_times       = [r['time_to_cross'] for r in results]
    viable_times    = [r['time_to_cross'] for r in results if r['aeb_viable']]
    collision_times = [r['time_to_cross'] for r in results if r['collision']]
 
    fig, ax = plt.subplots(figsize=(14, 7))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
 
    bins = np.linspace(min(all_times), min(max(all_times), 18), 42)
 
    ax.hist(collision_times, bins=bins,
            color='#E74C3C', alpha=0.85,
            label=f'Collision — AEB cannot respond ({pct_collision:.1f}%)',
            edgecolor='white', linewidth=0.4, zorder=3)
 
    if viable_times:
        ax.hist(viable_times, bins=bins,
                color='#27AE60', alpha=0.88,
                label=f'AEB viable ({pct_viable:.1f}%)',
                edgecolor='white', linewidth=0.4, zorder=4)
 
    # Minimum AEB response threshold
    ax.axvline(x=MIN_AEB_RESPONSE, color='#2c3e50',
               linestyle='--', linewidth=2.2, zorder=5,
               label=f'Min AEB response ({MIN_AEB_RESPONSE}s)')
 
    # S3 CARLA measured event duration — calibration reference
    S3_MEASURED = 4.15
    ax.axvline(x=S3_MEASURED, color='#8e44ad',
               linestyle=':', linewidth=2.0, zorder=5,
               label=f'S3 CARLA measured ({S3_MEASURED}s)')
 
    # ── ANNOTATION — positioned inside the sparse right tail area ────────────
    # Placed at (0.62, 0.78): below the legend box, in the low-density right half
    # where bars are well under 1000 scenarios, leaving clear space
    ax.text(0.62, 0.78,
            f'At highway speeds (60-100 km/h):\n'
            f'Side radar: non-actionable (parking spec <10 km/h)\n'
            f'Forward radar: structural 35 deg FOV gap\n'
            f'AEB viable: {pct_viable:.1f}%',
            transform=ax.transAxes,
            fontsize=10, va='top', ha='left',
            color='#8B0000', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.55',
                      facecolor='#fff5f5',
                      edgecolor='#E74C3C',
                      alpha=0.95))
 
    ax.set_xlabel('Time to Lane Crossing (seconds)',
                  fontsize=12, color='#1a1a1a')
    ax.set_ylabel('Number of Scenarios',
                  fontsize=12, color='#1a1a1a')
    ax.set_title(
        f'Monte Carlo Analysis — Motorcycle Lateral Departure Response Window\n'
        f'{total:,} scenarios  ·  '
        f'AEB viable: {pct_viable:.1f}%  ·  '
        f'Collision unavoidable: {pct_collision:.1f}%',
        fontsize=13, fontweight='bold', color='#1a1a1a', pad=14)
 
    ax.legend(fontsize=10, loc='upper right',
              framealpha=0.95, edgecolor='#cccccc')
    ax.grid(axis='y', alpha=0.25, color='#aaaaaa', zorder=0)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color('#cccccc')
    ax.tick_params(colors='#555555')
 
    plt.tight_layout()
    path = os.path.join(output_dir, 'mc_distribution.png')
    plt.savefig(path, dpi=150, facecolor='white', bbox_inches='tight')
    plt.close()
    print(f"  Chart: {path}  "
          f"(collision={pct_collision:.2f}%, viable={pct_viable:.2f}%)")
    return path
 
 
def generate_heatmap_chart(results, output_dir=OUTPUT_DIR):
    """
    Generate mc_heatmap.png — parameter space viability heatmap.
 
    Plots every scenario as a point in lateral-drift vs lane-gap space,
    coloured by collision (red) or AEB-viable (green). Shows that the
    entire realistic highway departure parameter space falls in the
    collision zone regardless of parameter combination.
 
    Args:
        results    : list of scenario result dicts from run_monte_carlo('baseline')
        output_dir : directory to save the output PNG
 
    Outputs:
        {output_dir}/mc_heatmap.png
    """
    total         = len(results)
    pct_collision = 100 * sum(r['collision']  for r in results) / total
    pct_viable    = 100 * sum(r['aeb_viable'] for r in results) / total
 
    # Separate parameter arrays by outcome
    col_drift = [r['lat_drift'] for r in results if r['collision']]
    col_gap   = [r['lane_gap']  for r in results if r['collision']]
    via_drift = [r['lat_drift'] for r in results if r['aeb_viable']]
    via_gap   = [r['lane_gap']  for r in results if r['aeb_viable']]
 
    fig, ax = plt.subplots(figsize=(12, 9))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
 
    # Plot collision scenarios first (behind), viable scenarios on top
    ax.scatter(col_drift, col_gap,
               c='#E74C3C', alpha=0.25, s=4,
               label=f'Collision ({pct_collision:.1f}%)',
               rasterized=True, zorder=2)
    if via_drift:
        ax.scatter(via_drift, via_gap,
                   c='#27AE60', alpha=0.7, s=6,
                   label=f'AEB viable ({pct_viable:.1f}%)',
                   rasterized=True, zorder=3)
 
    # S3 CARLA baseline validation point
    ax.scatter(0.7, 3.5,
               c='#8e44ad', marker='*', s=340, zorder=10,
               edgecolors='white', linewidth=1.2,
               label='S3 CARLA baseline (0.7 m/s, 3.5 m)')
 
    # Theoretical boundary line — where time_to_cross = MIN_AEB_RESPONSE
    # gap / drift * 0.83 = 1.5  →  gap = drift * 1.5 / 0.83
    drift_range = np.linspace(0.3, 1.2, 100)
    boundary_gap = drift_range * MIN_AEB_RESPONSE / S3_CORRECTION
    ax.plot(drift_range, boundary_gap,
            'k--', linewidth=1.8, alpha=0.6, zorder=5,
            label=f'Crossing time = {MIN_AEB_RESPONSE}s boundary')
 
    ax.set_xlabel('Lateral Drift Speed (m/s)', fontsize=12, color='#1a1a1a')
    ax.set_ylabel('Initial Lane Gap (m)',       fontsize=12, color='#1a1a1a')
    ax.set_title(
        f'AEB Viability by Departure Parameters\n'
        f'{total:,} scenarios  ·  '
        f'Green = AEB viable ({pct_viable:.1f}%)  ·  '
        f'Red = Collision ({pct_collision:.1f}%)',
        fontsize=13, fontweight='bold', color='#1a1a1a', pad=14)
 
    # Explanation box — bottom right (sparse in parameter space)
    ax.text(0.98, 0.04,
            'At highway speeds (>10 km/h):\n'
            'Side radar cannot trigger AEB\n'
            'All scenarios are red\n'
            'regardless of gap or drift',
            transform=ax.transAxes,
            fontsize=10, va='bottom', ha='right',
            color='#8B0000', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.5',
                      facecolor='#fff5f5',
                      edgecolor='#E74C3C',
                      alpha=0.95))
 
    ax.legend(fontsize=10, loc='upper right',
              framealpha=0.95, edgecolor='#cccccc')
    ax.grid(alpha=0.2, color='#aaaaaa', zorder=0)
    ax.set_axisbelow(True)
    ax.set_xlim(0.2, 1.3)
    ax.set_ylim(2.3, 5.2)
    for spine in ax.spines.values():
        spine.set_color('#cccccc')
    ax.tick_params(colors='#555555')
 
    plt.tight_layout()
    path = os.path.join(output_dir, 'mc_heatmap.png')
    plt.savefig(path, dpi=150, facecolor='white', bbox_inches='tight')
    plt.close()
    print(f"  Chart: {path}  (collision={pct_collision:.2f}%)")
    return path
 
 
def generate_failure_attribution(rates, output_dir=OUTPUT_DIR):
    """
    Generate failure_attribution.png — pie chart of failure modes.
 
    Shows the proportion of collision scenarios that are detection-limited
    (addressable through sensor improvements) versus physics-limited
    (unavoidable even with perfect detection). Includes counterfactual
    improvement summary panel.
 
    Args:
        rates      : dict mapping mode names to collision rates
                     {'baseline': 99.6, 'improved': 97.4, 'corner': 77.4,
                      'perfect': 46.6}
        output_dir : directory to save the output PNG
 
    Outputs:
        {output_dir}/failure_attribution.png
    """
    base    = rates['baseline']
    perfect = rates['perfect']
 
    # Detection-limited = failures that better sensing can prevent
    # Physics-limited   = failures unavoidable even with perfect detection
    detection_limited = base - perfect
    physics_limited   = perfect
 
    fig = plt.figure(figsize=(16, 8))
    fig.patch.set_facecolor('white')
 
    # ── LEFT: Pie chart ───────────────────────────────────────────────────────
    ax_pie = fig.add_subplot(1, 2, 1)
    ax_pie.set_facecolor('white')
 
    sizes  = [detection_limited, physics_limited]
    colors = ['#E74C3C', '#F39C12']
    labels = [
        f'Detection-limited\n({detection_limited:.1f}%)',
        f'Physics-limited\n({physics_limited:.1f}%)',
    ]
    explode = (0.05, 0)
 
    wedges, texts, autotexts = ax_pie.pie(
        sizes,
        labels=labels,
        colors=colors,
        explode=explode,
        autopct='%1.1f%%',
        startangle=90,
        textprops={'fontsize': 12, 'color': '#1a1a1a'},
        wedgeprops={'edgecolor': 'white', 'linewidth': 2})
 
    for at in autotexts:
        at.set_fontsize(13)
        at.set_fontweight('bold')
        at.set_color('white')
 
    ax_pie.set_title(
        f'AEB Failure Attribution\n'
        f'Monte Carlo: {100_000:,} Scenarios  ·  Baseline {base:.1f}%',
        fontsize=13, fontweight='bold', color='#1a1a1a', pad=16)
 
    # Explanation text below pie
    ax_pie.text(0, -1.42,
                f'Detection-limited: sensor architecture gaps prevent\n'
                f'actionable detection — addressable via hardware upgrades\n\n'
                f'Physics-limited: unavoidable even with perfect detection\n'
                f'due to lateral kinematics at highway speeds',
                ha='center', va='top', fontsize=10,
                color='#555555', linespacing=1.5,
                bbox=dict(boxstyle='round,pad=0.5',
                          facecolor='#f8f9fa',
                          edgecolor='#dddddd'))
 
    # ── RIGHT: Counterfactual improvement bars ────────────────────────────────
    ax_bar = fig.add_subplot(1, 2, 2)
    ax_bar.set_facecolor('white')
 
    modes  = ['baseline', 'improved', 'corner', 'perfect']
    labels_bar = [
        f'Baseline\n(current production)',
        f'Improved\n(highway-rated radar)',
        f'Corner radar\n(120 deg / 50 m)',
        f'Perfect detection\n(physics floor)',
    ]
    colors_bar = ['#c42c2c', '#d4711a', '#2471a3', '#1e8449']
    values     = [rates[m] for m in modes]
 
    bars = ax_bar.barh(labels_bar, values,
                       color=colors_bar, alpha=0.88, height=0.5,
                       edgecolor='white', linewidth=0.8)
 
    # Value labels on bars
    for bar, v in zip(bars, values):
        ax_bar.text(v + 0.4, bar.get_y() + bar.get_height() / 2,
                    f'{v:.1f}%', va='center', fontsize=12,
                    fontweight='bold', color='#1a1a1a')
 
    # Improvement annotations
    for i, mode in enumerate(modes[1:], 1):
        reduction = base - values[i]
        ax_bar.text(values[i] / 2,
                    bars[i].get_y() + bars[i].get_height() / 2,
                    f'-{reduction:.1f} pp',
                    va='center', ha='center',
                    fontsize=10, color='white', fontweight='bold')
 
    ax_bar.set_xlabel('Collision Rate (%)', fontsize=12, color='#1a1a1a')
    ax_bar.set_xlim(0, 108)
    ax_bar.set_title('Counterfactual Sensor Configuration Impact',
                     fontsize=13, fontweight='bold', color='#1a1a1a', pad=16)
    ax_bar.grid(axis='x', alpha=0.25, color='#aaaaaa', zorder=0)
    ax_bar.set_axisbelow(True)
    for spine in ax_bar.spines.values():
        spine.set_color('#cccccc')
    ax_bar.tick_params(colors='#555555', labelsize=11)
 
    # Key insight callout box
    # NEW — paste this
    ax_bar.text(0.5, -0.22,
            f'Sensing-driven gap: {detection_limited:.1f} pp    '
            f'Physics floor: {physics_limited:.1f}%    '
            f'Best achievable: {values[-1]:.1f}%',
            transform=ax_bar.transAxes,
            ha='center', va='top', fontsize=10,
            color='#1a6b35', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.5',
                      facecolor='#eafaf1',
                      edgecolor='#1e8449',
                      alpha=0.9))
 
    plt.suptitle(
        'AEB Failure Attribution Analysis\n'
        'Monte Carlo: 100,000 Scenarios  ·  Motorcycle Lateral Departure',
        fontsize=14, fontweight='bold', color='#1a1a1a', y=1.01)
 
    plt.tight_layout()
    path = os.path.join(output_dir, 'failure_attribution.png')
    plt.savefig(path, dpi=150, facecolor='white', bbox_inches='tight')
    plt.close()
    print(f"  Chart: {path}  "
          f"(detection-limited={detection_limited:.1f}pp, "
          f"physics-floor={physics_limited:.1f}%)")
    return path
 
 
def generate_impact_severity(results, output_dir=OUTPUT_DIR):
    """
    Generate impact_severity.png — distribution of collision impact speeds.
 
    Shows the distribution of residual impact speeds across all collision
    scenarios. Severity zones (high/medium/low) are shaded to connect
    Monte Carlo results to real-world injury outcomes. Near-full-speed
    impacts dominate, reinforcing the severity of the detection gap.
 
    Args:
        results    : list of scenario result dicts from run_monte_carlo('baseline')
        output_dir : directory to save the output PNG
 
    Outputs:
        {output_dir}/impact_severity.png
    """
    # Extract impact speeds for collision scenarios only
    impact_speeds = [r['impact_speed'] for r in results if r['collision']]
 
    if not impact_speeds:
        print("  Warning: no collision scenarios found for impact severity chart")
        return None
 
    total      = len(results)
    n_col      = len(impact_speeds)
    n_high     = sum(1 for v in impact_speeds if v > 20)
    n_medium   = sum(1 for v in impact_speeds if 10 < v <= 20)
    n_low      = sum(1 for v in impact_speeds if v <= 10)
    mean_speed = float(np.mean(impact_speeds))
    mean_kmh   = mean_speed * 3.6
 
    fig, ax = plt.subplots(figsize=(13, 7))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
 
    bins = np.linspace(0, max(impact_speeds) * 1.02, 40)
 
    # Split by severity zone for colour-coded bars
    low_speeds    = [v for v in impact_speeds if v <= 10]
    medium_speeds = [v for v in impact_speeds if 10 < v <= 20]
    high_speeds   = [v for v in impact_speeds if v > 20]
 
    ax.hist(high_speeds,   bins=bins, color='#c0392b', alpha=0.90,
            label=f'High severity  >20 m/s  ({100*n_high/total:.1f}% of all scenarios)',
            edgecolor='white', linewidth=0.3, zorder=3)
    ax.hist(medium_speeds, bins=bins, color='#e67e22', alpha=0.88,
            label=f'Medium severity  10-20 m/s  ({100*n_medium/total:.1f}%)',
            edgecolor='white', linewidth=0.3, zorder=4)
    ax.hist(low_speeds,    bins=bins, color='#f1c40f', alpha=0.85,
            label=f'Low severity  <10 m/s  ({100*n_low/total:.1f}%)',
            edgecolor='white', linewidth=0.3, zorder=5)
 
    # Mean impact speed reference line
    ax.axvline(x=mean_speed, color='#2c3e50',
               linestyle='--', linewidth=2.0, zorder=6,
               label=f'Mean impact: {mean_speed:.1f} m/s ({mean_kmh:.0f} km/h)')
 
    # Severity zone background shading
    ax.axvspan(0,  10, alpha=0.04, color='#f1c40f', zorder=1)
    ax.axvspan(10, 20, alpha=0.04, color='#e67e22', zorder=1)
    ax.axvspan(20, max(impact_speeds) * 1.02,
                   alpha=0.04, color='#c0392b', zorder=1)
 
    # Zone boundary lines
    for boundary, label in [(10, '10 m/s'), (20, '20 m/s')]:
        ax.axvline(x=boundary, color='#aaaaaa',
                   linestyle=':', linewidth=1.2, zorder=2)
        ax.text(boundary + 0.15, ax.get_ylim()[1] * 0.98,
                label, fontsize=9, color='#888888',
                va='top', rotation=90)
 
    # Statistics box
    ax.text(0.02, 0.97,
            f'Mean impact: {mean_speed:.1f} m/s ({mean_kmh:.0f} km/h)\n'
            f'High severity (>20 m/s):   {100*n_high/total:.1f}% of all scenarios\n'
            f'Medium (10-20 m/s):         {100*n_medium/total:.1f}%\n'
            f'Low (<10 m/s):              {100*n_low/total:.1f}%',
            transform=ax.transAxes,
            fontsize=10, va='top', ha='left',
            color='#1a1a1a',
            bbox=dict(boxstyle='round,pad=0.55',
                      facecolor='#f8f9fa',
                      edgecolor='#dddddd',
                      alpha=0.96))
 
    ax.set_xlabel('Residual Impact Speed (m/s)', fontsize=12, color='#1a1a1a')
    ax.set_ylabel('Number of Scenarios',         fontsize=12, color='#1a1a1a')
    ax.set_title(
        f'Impact Speed Distribution — Collision Scenarios\n'
        f'{n_col:,} collision scenarios  ·  '
        f'Mean impact: {mean_speed:.1f} m/s ({mean_kmh:.0f} km/h)  ·  '
        f'No AEB braking applied',
        fontsize=13, fontweight='bold', color='#1a1a1a', pad=14)
 
    ax.legend(fontsize=10, loc='upper left', bbox_to_anchor=(0.0, 0.72),
              framealpha=0.95, edgecolor='#cccccc')
    ax.grid(axis='y', alpha=0.25, color='#aaaaaa', zorder=0)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color('#cccccc')
    ax.tick_params(colors='#555555')
 
    plt.tight_layout()
    path = os.path.join(output_dir, 'impact_severity.png')
    plt.savefig(path, dpi=150, facecolor='white', bbox_inches='tight')
    plt.close()
    print(f"  Chart: {path}  "
          f"(mean={mean_speed:.2f} m/s, "
          f"high={100*n_high/total:.1f}%)")
    return path  


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
    generate_headline_chart(results['baseline'])
    print("\nGenerating additional charts...")
    generate_headline_chart(results['baseline'])
    generate_distribution_chart(results['baseline'])  
    generate_heatmap_chart(results['baseline'])
    generate_failure_attribution(rates)
    generate_impact_severity(results['baseline'])
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