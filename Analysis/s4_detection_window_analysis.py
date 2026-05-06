"""
s4_detection_window_analysis.py
================================
Detection window comparison chart and table generation.

Author:  Tejas Manjunath
Project: Lateral Intrusion ADAS Gap — AEB Blind Zone Study

What this script does:
  Generates publication-ready visualizations comparing AEB detection
  windows across Euro NCAP tested scenarios and the motorcycle lateral
  departure scenario validated in CARLA S3 simulation. Produces bar
  chart and comparison table using validated CARLA physics results.

Outputs (saved to s4_output1/):
  detection_window_chart.png — bar chart showing detection windows
  detection_window_table.png — comparison table with severity data

Key validated findings (from S3 CARLA simulation):
  Departure:      t=5.00s
  Collision:      t=9.15s (physics impact)
  Event duration: 4.15s
  Both vehicles:  80 km/h (waypoint-controlled)
  Lane gap:       3.50m (constant during stable phase)

Sensor findings:
  Forward radar:  FOV gap — MC outside 35° cone throughout event
  Side radar:     Detection present but non-actionable (parking spec)
  Rear radar:     Geometry gap — first hit at collision, not departure

Euro NCAP comparison:
  CCRs (rear stationary):    3.8s window → AEB viable
  CCRm (rear moving):        2.9s window → AEB viable
  CPNA (pedestrian):         2.1s window → AEB viable
  LCR + Motorcycle (fwd):    0.0s window → AEB NOT viable
  LCR + Motorcycle (side):   Non-actionable → AEB NOT viable

Requirements:
  Python 3.10+
  matplotlib, numpy
  pip install matplotlib numpy

Usage:
  python s4_detection_window_analysis.py
"""

# ── IMPORTS ────────────────────────────────────────────────────────────────
import os

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for server environments
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# ── CONFIGURATION ──────────────────────────────────────────────────────────
OUTPUT_DIR = 's4_detection_window_analysis'
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── S3 CARLA SIMULATION RESULTS (validated) ───────────────────────────────
# Results from S3_ego_mc.py physics-based simulation
# Town04 highway, 80 km/h, BMW Grand Tourer vs Kawasaki Ninja
S3_RESULTS = {
    'departure_time': 5.00,  # seconds — motorcycle begins lateral drift
    'collision_time': 9.15,  # seconds — physics-based impact
    'event_duration': 4.15,  # seconds — departure to collision window
    'ego_speed_kmh': 80,  # km/h — ego vehicle speed (waypoint-controlled)
    'moto_speed_kmh': 80,  # km/h — motorcycle speed (waypoint-controlled)
    'lane_gap_m': 3.50,  # meters — initial lateral separation
    'fwd_radar': 'False positives only — road infrastructure reflections. '
                 'MC outside 35° forward FOV throughout event.',
    'side_radar': 'Detection present but non-actionable. '
                  'Parking-assist spec (8m, <10 km/h). '
                  'MC closed from 3.25m to 0m over 4.15s.',
    'rear_radar': 'Geometry gap confirmed. '
                  'First hit t=13.3s coincides with collision, not departure.',
}


# ── EURO NCAP SCENARIOS (comparison baseline) ─────────────────────────────
# Detection window data for Euro NCAP tested scenarios plus LCR motorcycle gap
EURO_NCAP_SCENARIOS = [
    {
        'name': 'CCRs\n(Rear Stationary)',
        'window': 3.8,  # seconds — detection window for AEB intervention
        'tested': True,  # Included in Euro NCAP AEB protocol
        'color': '#2ecc71',
    },
    {
        'name': 'CCRm\n(Rear Moving)',
        'window': 2.9,
        'tested': True,
        'color': '#27ae60',
    },
    {
        'name': 'CPNA\n(Pedestrian Crossing)',
        'window': 2.1,
        'tested': True,
        'color': '#f39c12',
    },
    {
        'name': 'CBNA\n(Cyclist Crossing)',
        'window': 1.9,
        'tested': True,
        'color': '#e67e22',
    },
    {
        'name': 'LCR + Motorcycle\n(Forward Radar)',
        'window': 0.0,  # No detection — structural FOV gap
        'tested': False,  # NOT in Euro NCAP protocol
        'color': '#e74c3c',
    },
    {
        'name': 'LCR + Motorcycle\n(Side Radar)',
        'window': 0.0,  # Detection present but non-actionable
        'tested': False,
        'color': '#c0392b',
        'note': 'Non-actionable\n(parking spec)'
    },
]

# Minimum time required for AEB system to respond (industry standard)
MIN_AEB_RESPONSE = 1.5  # seconds


# ── DETECTION WINDOW CHART GENERATION ──────────────────────────────────────
def generate_detection_window_chart():
    """
    Generate bar chart comparing detection windows across scenarios.
    
    Produces horizontal comparison of Euro NCAP tested scenarios against
    motorcycle lateral departure. Shows detection window in seconds with
    minimum AEB response threshold line.
    
    Returns:
        Path to saved chart PNG
    """
    fig, ax = plt.subplots(figsize=(14, 7))

    # Extract scenario data for plotting
    names = [s['name'] for s in EURO_NCAP_SCENARIOS]
    windows = [s['window'] for s in EURO_NCAP_SCENARIOS]
    colors = [s['color'] for s in EURO_NCAP_SCENARIOS]
    tested = [s['tested'] for s in EURO_NCAP_SCENARIOS]

    # Create bar chart
    x = np.arange(len(names))
    bars = ax.bar(x, windows, color=colors, width=0.6,
                  edgecolor='white', linewidth=1.5, zorder=3)

    # Add minimum AEB response threshold line
    ax.axhline(y=MIN_AEB_RESPONSE, color='#2c3e50',
               linestyle='--', linewidth=2.0, zorder=4)
    ax.axhspan(0, MIN_AEB_RESPONSE, alpha=0.08, color='red')

    # Add value labels and gap annotations
    for bar, window, is_tested, scenario in zip(bars, windows, tested, EURO_NCAP_SCENARIOS):
        if window > 0:
            # Normal scenario — show detection window value
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + 0.05,
                    f'{window:.1f}s',
                    ha='center', va='bottom',
                    fontsize=11, fontweight='bold', color='#2c3e50')
        else:
            # Gap scenario — show small red indicator bar
            ax.bar(bar.get_x() + bar.get_width() / 2,
                   0.15, width=0.6, color='#c0392b', zorder=5)

            # Different labels for forward vs side radar gaps
            if 'Forward' in scenario['name']:
                label = 'NO\nDETECTION\n(FOV gap)'
            else:
                label = 'NON-\nACTIONABLE\n(parking spec)'

            ax.text(bar.get_x() + bar.get_width() / 2,
                    0.20,
                    label,
                    ha='center', va='bottom',
                    fontsize=7, fontweight='bold',
                    color='#c0392b',
                    bbox=dict(boxstyle='round,pad=0.3',
                              facecolor='white',
                              edgecolor='#c0392b',
                              alpha=0.9))

    # Mark scenarios not in Euro NCAP protocol
    for i, (is_tested, window) in enumerate(zip(tested, windows)):
        if not is_tested:
            ax.text(x[i], 0.85,
                    '★ NOT IN\nEURO NCAP',
                    ha='center', va='bottom',
                    fontsize=7, color='#c0392b', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.2',
                              facecolor='#ffeaea',
                              edgecolor='#c0392b',
                              alpha=0.9))

    # Configure axes and labels
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=10)
    ax.set_ylabel('Detection Window (seconds)', fontsize=12)
    ax.set_title(
        'AEB Sensor Detection Windows by Scenario Type\n'
        'Simulation Evidence: Motorcycle Leaving-Carriageway vs Euro NCAP Tested Scenarios',
        fontsize=13, fontweight='bold', pad=15)
    ax.set_ylim(0, 4.8)
    ax.grid(axis='y', alpha=0.3, zorder=0)
    ax.set_axisbelow(True)

    # Add legend
    tested_patch = mpatches.Patch(color='#27ae60', alpha=0.8,
                                  label='Euro NCAP Tested Scenario')
    untested_patch = mpatches.Patch(color='#e74c3c',
                                    label='NOT in Euro NCAP Protocol')
    ax.legend(handles=[tested_patch, untested_patch,
                       plt.Line2D([0], [0], color='#2c3e50',
                                  linestyle='--', linewidth=2,
                                  label=f'Min AEB Response ({MIN_AEB_RESPONSE}s)')],
              loc='upper right', fontsize=10)

    # Add CARLA simulation annotation
    ax.annotate(
        'CARLA Simulation (physics-based):\n'
        'Both vehicles: 80 km/h\n'
        'Departure onset: t=5.0s\n'
        'Collision: t=9.15s\n'
        'Event window: 4.15s\n'
        'Fwd radar: FOV gap — no MC detection\n'
        'Side radar: detection non-actionable',
        xy=(4.5, 0.3),
        xytext=(3.6, 2.9),
        fontsize=8.5,
        bbox=dict(boxstyle='round,pad=0.5',
                  facecolor='#ffeaa7',
                  edgecolor='#e17055',
                  alpha=0.95),
        arrowprops=dict(arrowstyle='->', color='#e17055'))

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'detection_window_chart.png')
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"Chart saved: {path}")
    return path


# ── DETECTION WINDOW TABLE GENERATION ──────────────────────────────────────
def generate_summary_table():
    """
    Generate comparison table for detection windows and AEB viability.
    
    Creates publication-ready table comparing Euro NCAP tested scenarios
    with motorcycle lateral departure, including German crash severity
    data and AEB viability assessment.
    
    Returns:
        Path to saved table PNG
    """
    fig, ax = plt.subplots(figsize=(15, 5.5))
    ax.axis('off')

    # Define table structure
    columns = ['Scenario', 'Severity Rate\n(German Data)',
               'Detection Window', 'Min AEB\nRequired',
               'AEB Viable?', 'Euro NCAP\nTested?']

    # Table data rows
    rows = [
        ['CCRs — Rear Stationary', 'N/A', '3.8 s', '1.5 s', '✓ Yes', '✓ Yes'],
        ['CCRm — Rear Moving', 'N/A', '2.9 s', '1.5 s', '✓ Yes', '✓ Yes'],
        ['CPNA — Pedestrian Crossing', '23.4%', '2.1 s', '1.5 s', '✓ Yes', '✓ Yes'],
        ['CBNA — Cyclist Crossing', 'N/A', '1.9 s', '1.5 s', '✓ Yes', '✓ Yes'],
        ['LCR + Motorcycle (Fwd Radar)', '44.1%', '0 s — FOV gap', '1.5 s', '✗ No', '✗ No'],
        ['LCR + Motorcycle (Side Radar)', '44.1%', 'Non-actionable *', '1.5 s', '✗ No', '✗ No'],
    ]

    # Create table
    table = ax.table(cellText=rows, colLabels=columns,
                     loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2.2)

    # Style header row
    for j in range(len(columns)):
        table[0, j].set_facecolor('#2c3e50')
        table[0, j].set_text_props(color='white', fontweight='bold')

    # Style data rows — highlight motorcycle gap scenarios
    for i, row in enumerate(rows):
        is_gap = 'Motorcycle' in row[0]
        for j in range(len(columns)):
            if is_gap:
                table[i + 1, j].set_facecolor('#ffd5d5')
            else:
                table[i + 1, j].set_facecolor(
                    '#f8f9fa' if i % 2 == 0 else 'white')

    # Add title
    ax.set_title(
        'Detection Window Comparison — Euro NCAP Tested vs Motorcycle Leaving-Carriageway\n'
        'Simulation: CARLA 0.9.16 | Physics-based collision | Data: 1.53M German Crash Records',
        fontsize=12, fontweight='bold', pad=20)

    # Add footnote explaining side radar non-actionable status
    ax.text(0.5, -0.08,
            '* Side radar detection present (MC closed 3.25m → 0m over 4.15s) but non-actionable: '
            'parking-assist specification designed for <10 km/h, cannot trigger AEB at 80 km/h highway speeds.',
            transform=ax.transAxes, ha='center', va='top',
            fontsize=8.5, color='#555555', style='italic')

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'detection_window_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"Table saved: {path}")
    return path


# ── FINDINGS SUMMARY ───────────────────────────────────────────────────────
def print_findings():
    """
    Print comprehensive findings summary to console.
    
    Outputs validated CARLA simulation results, sensor-by-sensor analysis,
    Euro NCAP comparison, and final conclusions regarding the AEB validation
    gap for motorcycle lateral departure scenarios.
    """
    print("\n" + "=" * 70)
    print("S4 FINAL FINDINGS — DETECTION WINDOW ANALYSIS")
    print("=" * 70)
    print(f"""
SIMULATION EVIDENCE (CARLA 0.9.16 | Physics-based | Town04 Highway):
  Scenario:            LCR — Motorcycle Lateral Departure
  Both vehicles:       {S3_RESULTS['ego_speed_kmh']} km/h (exact, waypoint-controlled)
  Initial lane gap:    {S3_RESULTS['lane_gap_m']:.2f} m (constant during stable phase)
  Departure onset:     t={S3_RESULTS['departure_time']:.2f}s
  Physics collision:   t={S3_RESULTS['collision_time']:.2f}s
  Event duration:      {S3_RESULTS['event_duration']:.2f}s

SENSOR FINDINGS:

  1. FORWARD RADAR (35° FOV, 150m range)
     Finding: {S3_RESULTS['fwd_radar']}
     Implication: Structural FOV gap. Lateral threats are architecturally
     invisible to forward-facing radar regardless of range or sensitivity.

  2. SIDE RADAR (120° FOV, 8m range — parking-assist specification)
     Finding: {S3_RESULTS['side_radar']}
     Implication: Detection present but non-actionable. Production BSD
     systems at this specification cannot trigger AEB intervention at
     highway speeds. By the time MC enters ego's lane boundary, 4.15
     seconds have elapsed with no intervention window remaining.

  3. REAR RADAR (150° FOV, 80m range)
     Finding: {S3_RESULTS['rear_radar']}
     Implication: Geometry gap. MC approaches from beside, not behind.
     Rear radar architecture cannot cover lateral approach trajectories.

COMPARISON TO EURO NCAP TESTED SCENARIOS:
  CCRs (rear stationary):    3.8s detection window  → AEB viable
  CCRm (rear moving):        2.9s detection window  → AEB viable
  CPNA (pedestrian):         2.1s detection window  → AEB viable
  LCR + Motorcycle (fwd):    0.0s (FOV gap)         → AEB NOT viable
  LCR + Motorcycle (side):   Non-actionable          → AEB NOT viable

CONCLUSION:
  During a physics-simulated motorcycle leaving-carriageway event at
  80 km/h with a 4.15-second collision window, no sensor modality in
  a standard production AEB suite provided an actionable departure
  warning.

  This scenario class — which causes 44.1% severe injury rates across
  1.53M German crash records (2.37× national baseline) — sits entirely
  outside the validated envelope of current AEB sensor architecture
  AND outside current Euro NCAP AEB test protocols.

  The forward radar has a structural field-of-view gap for lateral
  threats. The side radar specification is insufficient for highway-speed
  intervention. The rear radar has a geometric blind spot for lateral
  approach trajectories.

  This is not a sensor failure. It is a validation scope failure.
    """)
    print("=" * 70)


# ── MAIN ───────────────────────────────────────────────────────────────────
def main():
    """
    Execute complete visualization pipeline.
    
    Generates detection window bar chart, comparison table, and prints
    comprehensive findings summary based on validated S3 CARLA results.
    """
    print("=" * 70)
    print("S4: Detection Window Analysis — Updated with Physics Simulation")
    print("=" * 70)

    chart_path = generate_detection_window_chart()
    table_path = generate_summary_table()
    print_findings()

    print(f"\nOutputs saved to {OUTPUT_DIR}/")
    print(f"  {chart_path}")
    print(f"  {table_path}")


if __name__ == '__main__':
    main()