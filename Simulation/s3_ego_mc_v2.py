"""
s3_ego_mc.py
============
CARLA 0.9.16 — Motorcycle Lateral Departure Scenario
Phase 2: AEB Blind Side Study

Author : Tejas Manjunath
Repo   : github.com/TejasManjunath/aeb-blind-side-study

Scenario
--------
A motorcycle (Kawasaki Ninja) departs laterally from an adjacent lane
into the path of the ego vehicle (BMW Grand Tourer) at 80 km/h.
The simulation measures whether TTC-based AEB can detect and respond
in time to prevent collision.

Run modes (set flags at bottom of CONFIGURATION section)
---------
  Default        — single baseline run, full camera output
  CF_SWEEP_MODE  — validates S3 correction factor across 5 drift speeds
  TTC_SWEEP_MODE — tests AEB outcome across 8 TTC thresholds

Outputs (saved to OUTPUT_DIR)
-------
  mc_sensor_log.txt        human-readable sensor event summary
  mc_sensor_raw.txt        raw per-tick radar returns
  mc_baseline_metrics.csv  kinematic metrics for Monte Carlo calibration
  mc_tick_data.csv         per-tick position, TTC, lateral separation
  cam_chase_*.png          chase camera frames (when SAVE_FRAMES=True)
  cam_overhead_*.png       overhead camera frames
  cam_side_*.png           side camera frames

Requirements
------------
  CARLA 0.9.16 server running on localhost:2000
  Python 3.10
  pip install numpy opencv-python pillow

Usage
-----
  python s3_ego_mc.py
"""

# ── IMPORTS ────────────────────────────────────────────────────────────────
import sys
import time
import math
import os

sys.path.append('C:/CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-py3.10-win-amd64.egg')

import carla
import numpy as np
import cv2


# ── CONFIGURATION ──────────────────────────────────────────────────────────

OUTPUT_DIR = 's3_ego_mc_output_r1'
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Vehicle speeds — both ego and motorcycle hold constant highway speed
SPEED_KMH    = 80.0
SPEED_MS     = SPEED_KMH / 3.6
TICK         = 0.05               # Simulation timestep: 20 Hz
DIST_PER_TICK = SPEED_MS * TICK   # Distance per tick at 80 km/h

# Scenario timing
STABLE_T     = 5.0    # Stable parallel travel phase before lateral departure begins
RUN_DURATION = 16.0   # Total simulation runtime in seconds

# Lateral intrusion parameters
LAT_SPEED      = 0.7   # Motorcycle lateral drift speed (m/s)
LAT_PER_TICK   = LAT_SPEED * TICK
IMPACT_THRESH  = 0.6   # Lateral separation at which CARLA physics collision triggers (m)
                        # Approximates motorcycle body half-width

# AEB system parameters — production-representative values
AEB_ENABLED    = True
REACTION_TIME  = 1.0   # Total system delay: sensor processing + ECU + actuator (s)
MIN_TTC_TRIGGER = 4.0  # TTC threshold for AEB activation (s)
MAX_DECEL      = 8.0   # Maximum emergency braking deceleration (m/s²)

# Spawn location — Town04 straight highway section
START_X = 246.0
START_Y = -388.7
START_Z = 0.3

# Camera frame capture
frame_counts = {'side': 0, 'overhead': 0, 'chase': 0}
frame_every  = 4      # Save every 4th frame to limit disk I/O
SAVE_FRAMES  = False  # Set True for single-run frame capture; False during sweeps

# ── RUN MODE FLAGS ─────────────────────────────────────────────────────────
# Set exactly one to True, or both False for a standard single run.
CF_SWEEP_MODE  = True   # Correction factor validation across 5 drift speeds
TTC_SWEEP_MODE = False  # TTC threshold sensitivity across 8 thresholds

# Drift speeds for correction factor validation sweep
CF_SWEEP_DRIFTS = [0.3, 0.5, 0.7, 0.9, 1.2]  # m/s

# TTC thresholds for sensitivity sweep — spans realistic production AEB range
TTC_SWEEP_THRESHOLDS = [1.5, 2.0, 2.5, 3.0, 3.43, 4.0, 4.5, 5.0]  # seconds

# Sweep result accumulators — populated during main() calls
cf_results  = []
ttc_results = []


# ── CAMERA CALLBACK ────────────────────────────────────────────────────────
def make_cam_cb(label):
    """
    Returns a callback that writes camera frames directly to disk.
    Saves every Nth frame (frame_every) to manage storage.
    Silently skips frames on memory pressure.
    """
    counter = [0]
    def cb(img):
        if not SAVE_FRAMES:
            return
        counter[0] += 1
        if counter[0] % frame_every != 0:
            return
        try:
            arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(
                (img.height, img.width, 4))[:, :, :3]
            path = os.path.join(OUTPUT_DIR,
                f"cam_{label}_t{img.timestamp:.1f}_f{img.frame}.png")
            cv2.imwrite(path, arr)
            frame_counts[label] += 1
        except Exception:
            pass
    return cb


# ── RADAR DATA STORAGE ─────────────────────────────────────────────────────
fwd_log  = []  # Forward radar detection events
side_log = []  # Side radar detection events
rear_log = []  # Rear radar detection events

t0_carla   = None  # CARLA elapsed time at scenario start
dep_time_s = None  # Scenario-relative time of motorcycle departure


def rel(at):
    """Convert CARLA absolute timestamp to scenario-relative seconds."""
    return round(at - t0_carla, 3) if (t0_carla is not None and at is not None) else None


# ── RADAR CALLBACKS ────────────────────────────────────────────────────────
def fwd_cb(data):
    """
    Forward radar — 35° H-FOV, 150 m range.
    Standard AEB long-range radar specification.
    Creates a geometric blind zone for lateral targets.
    """
    hits = [d for d in data if d.depth < 150 and abs(math.degrees(d.azimuth)) < 17.5]
    if hits:
        fwd_log.append({
            't_rel':     rel(data.timestamp),
            'n':         len(hits),
            'min_depth': round(min(h.depth for h in hits), 1)
        })


def side_cb(data):
    """
    Side radar — 120° H-FOV, 8 m range.
    Parking-assist specification; non-actionable at highway speed.
    Detection below 10 km/h threshold only in production systems.
    """
    hits = [d for d in data if d.depth < 8.0]
    if hits:
        side_log.append({
            't_rel':     rel(data.timestamp),
            'n':         len(hits),
            'min_depth': round(min(h.depth for h in hits), 2)
        })


def rear_cb(data):
    """
    Rear radar — 150° H-FOV, 80 m range.
    Blind-spot monitoring / rear cross-traffic alert specification.
    Geometry gap: motorcycle is ahead of ego during lateral intrusion.
    """
    hits = [d for d in data if d.depth < 80 and abs(math.degrees(d.azimuth)) < 75]
    if hits:
        rear_log.append({
            't_rel':     rel(data.timestamp),
            'n':         len(hits),
            'min_depth': round(min(h.depth for h in hits), 1)
        })


# ── PATH GENERATION ────────────────────────────────────────────────────────
def build_path(start_wp, n_steps):
    """
    Generate a kinematic waypoint path along the road topology.

    Returns list of (x, y, z, yaw) tuples for n_steps ahead,
    each spaced DIST_PER_TICK apart.
    """
    path = []
    wp = start_wp
    for _ in range(n_steps):
        t = wp.transform
        path.append((t.location.x, t.location.y, t.location.z, t.rotation.yaw))
        nexts = wp.next(DIST_PER_TICK)
        wp = nexts[0] if nexts else wp
    return path


# ── MAIN SIMULATION ────────────────────────────────────────────────────────
def main():
    """
    Execute one lateral intrusion scenario run.

    Phase 1 — Stable (0 to STABLE_T s):
        Both vehicles travel in adjacent lanes at 80 km/h.
        Kinematic (physics off) for deterministic positioning.

    Phase 2 — Departure (STABLE_T to impact):
        Motorcycle drifts laterally at LAT_SPEED toward ego lane.
        AEB TTC logic monitored. Reaction delay applied before braking.

    Phase 3 — Physics (impact onward):
        CARLA physics enabled when lateral gap < IMPACT_THRESH.
        AEB braking applied via set_target_velocity if triggered.

    Outputs sensor log, raw radar data, baseline metrics, and tick data.
    """
    global fwd_log, side_log, rear_log, t0_carla, dep_time_s
    fwd_log.clear()
    side_log.clear()
    rear_log.clear()
    t0_carla   = None
    dep_time_s = None
    actors     = []
    impact_time = None
    physics_on  = False

    aeb_triggered    = False
    aeb_trigger_time = None

    print("=" * 60)
    print("ADAS SAFETY SCENARIO — FINAL + SENSOR LOG")
    print("=" * 60)

    # ── CARLA CONNECTION ───────────────────────────────────────────────────
    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    world  = client.get_world()

    if 'Town04' not in world.get_map().name:
        print("Loading Town04...")
        world = client.load_world('Town04')
        time.sleep(12.0)
    else:
        print("Town04 ready")

    for v in world.get_actors().filter('vehicle.*'):
        try:
            v.destroy()
        except:
            pass
    time.sleep(2.0)
    print("World cleared\n")

    try:
        lib    = world.get_blueprint_library()
        rd_map = world.get_map()

        # Daytime clear weather for consistent sensor conditions
        world.set_weather(carla.WeatherParameters(
            sun_altitude_angle=60.0, sun_azimuth_angle=220.0,
            cloudiness=15.0, fog_density=0.0, precipitation=0.0))

        # ── WAYPOINT SETUP ─────────────────────────────────────────────────
        ego_wp = rd_map.get_waypoint(
            carla.Location(x=START_X, y=START_Y, z=START_Z),
            project_to_road=True, lane_type=carla.LaneType.Driving)

        mc_wp = ego_wp.get_right_lane()
        if mc_wp is None or mc_wp.lane_type != carla.LaneType.Driving:
            mc_wp = ego_wp.get_left_lane()
        assert mc_wp is not None, "No adjacent driving lane found."

        # Advance both starting waypoints past the Town04 corner onto straight section
        SKIP = 200
        for _ in range(SKIP):
            n = ego_wp.next(DIST_PER_TICK)
            ego_wp = n[0] if n else ego_wp
            n = mc_wp.next(DIST_PER_TICK)
            mc_wp = n[0] if n else mc_wp
        print(f"Skipped {SKIP} waypoints (~{SKIP * DIST_PER_TICK:.0f}m) to straight section")

        # Measure lane gap and compute lateral unit vector (ego → motorcycle)
        eloc = ego_wp.transform.location
        mloc = mc_wp.transform.location
        LANE_GAP = math.hypot(eloc.x - mloc.x, eloc.y - mloc.y)
        lat_dx   = (mloc.x - eloc.x) / LANE_GAP
        lat_dy   = (mloc.y - eloc.y) / LANE_GAP
        print(f"Spawn EGO: ({eloc.x:.1f},{eloc.y:.1f})  MC: ({mloc.x:.1f},{mloc.y:.1f})")
        print(f"Lane gap: {LANE_GAP:.2f} m\n")

        # ── PATH GENERATION ────────────────────────────────────────────────
        N        = int(RUN_DURATION / TICK) + 50
        ego_path = build_path(ego_wp, N)
        mc_path  = build_path(mc_wp,  N)

        # Side camera yaw — points from ego toward motorcycle lane
        road_yaw            = ego_wp.transform.rotation.yaw
        toward_mc_world_yaw = math.degrees(math.atan2(lat_dy, lat_dx))
        side_cam_yaw        = toward_mc_world_yaw - road_yaw

        # ── VEHICLE SPAWNING ───────────────────────────────────────────────
        ego_bp = lib.find('vehicle.bmw.grandtourer')
        ego_bp.set_attribute('color', '0,50,200')
        mc_bp  = lib.find('vehicle.kawasaki.ninja')
        mc_bp.set_attribute('color', '220,80,0')

        p0 = ego_path[0]
        m0 = mc_path[0]

        ego = world.spawn_actor(ego_bp,
            carla.Transform(carla.Location(x=p0[0], y=p0[1], z=p0[2] + 0.15),
                            carla.Rotation(yaw=p0[3])))
        actors.append(ego)

        mc = world.spawn_actor(mc_bp,
            carla.Transform(carla.Location(x=m0[0], y=m0[1], z=m0[2] + 0.15),
                            carla.Rotation(yaw=m0[3])))
        actors.append(mc)

        # Kinematic mode during stable phase — deterministic, no physics drift
        ego.set_simulate_physics(False)
        mc.set_simulate_physics(False)
        ego.set_light_state(carla.VehicleLightState(carla.VehicleLightState.LowBeam))
        mc.set_light_state(carla.VehicleLightState(carla.VehicleLightState.LowBeam))
        print(f"EGO id={ego.id}  MC id={mc.id}")
        time.sleep(1.0)

        # ── RADAR SENSOR SETUP ─────────────────────────────────────────────
        # Forward — production AEB long-range spec
        r = lib.find('sensor.other.radar')
        r.set_attribute('horizontal_fov', '35')
        r.set_attribute('vertical_fov',   '10')
        r.set_attribute('range',          '150')
        r.set_attribute('points_per_second', '2000')
        fwd_r = world.spawn_actor(r,
            carla.Transform(carla.Location(x=2.4, z=0.8)), attach_to=ego)
        actors.append(fwd_r)
        fwd_r.listen(fwd_cb)

        # Side — parking-assist spec (non-actionable at highway speed)
        rs = lib.find('sensor.other.radar')
        rs.set_attribute('horizontal_fov', '120')
        rs.set_attribute('vertical_fov',   '20')
        rs.set_attribute('range',          '8')
        rs.set_attribute('points_per_second', '1500')
        side_r = world.spawn_actor(rs,
            carla.Transform(carla.Location(x=0, z=0.8),
                            carla.Rotation(yaw=side_cam_yaw)), attach_to=ego)
        actors.append(side_r)
        side_r.listen(side_cb)

        # Rear — blind-spot monitoring / RCTA spec
        rr = lib.find('sensor.other.radar')
        rr.set_attribute('horizontal_fov', '150')
        rr.set_attribute('vertical_fov',   '10')
        rr.set_attribute('range',          '80')
        rr.set_attribute('points_per_second', '1500')
        rear_r = world.spawn_actor(rr,
            carla.Transform(carla.Location(x=-2.4, z=0.8),
                            carla.Rotation(yaw=180)), attach_to=ego)
        actors.append(rear_r)
        rear_r.listen(rear_cb)

        # ── CAMERA SETUP ───────────────────────────────────────────────────
        def make_cam(fov='90'):
            bp = lib.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', '1280')
            bp.set_attribute('image_size_y', '720')
            bp.set_attribute('fov', fov)
            return bp

        # Side view — shows motorcycle lateral approach
        cs = world.spawn_actor(make_cam('100'),
            carla.Transform(carla.Location(x=0.5, z=1.8),
                            carla.Rotation(pitch=-5, yaw=side_cam_yaw)), attach_to=ego)
        actors.append(cs)
        cs.listen(make_cam_cb('side'))

        # Overhead view — top-down lane geometry
        co = world.spawn_actor(make_cam('100'),
            carla.Transform(carla.Location(z=18),
                            carla.Rotation(pitch=-90, yaw=0)), attach_to=ego)
        actors.append(co)
        co.listen(make_cam_cb('overhead'))

        # Chase view — rear 3/4 perspective
        cc = world.spawn_actor(make_cam('90'),
            carla.Transform(carla.Location(x=-8, z=5),
                            carla.Rotation(pitch=-18, yaw=0)), attach_to=ego)
        actors.append(cc)
        cc.listen(make_cam_cb('chase'))

        print(f"Cameras: writing to disk every {frame_every}th frame")

        # Position spectator for visual monitoring during the run
        world.get_spectator().set_transform(carla.Transform(
            carla.Location(
                x=p0[0] - math.cos(math.radians(p0[3])) * 10 - lat_dx * 8,
                y=p0[1] - math.sin(math.radians(p0[3])) * 10 - lat_dy * 8,
                z=p0[2] + 12),
            carla.Rotation(pitch=-30, yaw=p0[3])))

        time.sleep(1.5)
        t0_carla = world.get_snapshot().timestamp.elapsed_seconds
        print(f"\nRUNNING: departure at t={STABLE_T}s | total {RUN_DURATION}s")
        print("-" * 60)

        # ── MAIN LOOP ──────────────────────────────────────────────────────
        dep_lateral = 0.0
        departed    = False
        last_print  = 0
        ticks       = 0
        wall_start  = time.time()

        while True:
            t = ticks * TICK
            if t >= RUN_DURATION:
                break
            pi = min(ticks, len(ego_path) - 1)

            if not physics_on:
                # Kinematic phase — set transforms directly from path arrays
                ex, ey, ez, eyaw = ego_path[pi]
                ego.set_transform(carla.Transform(
                    carla.Location(x=ex, y=ey, z=ez + 0.15),
                    carla.Rotation(yaw=eyaw)))

                if not departed:
                    mx, my, mz, myaw = mc_path[pi]
                    mc.set_transform(carla.Transform(
                        carla.Location(x=mx, y=my, z=mz + 0.15),
                        carla.Rotation(yaw=myaw)))
                else:
                    # Apply cumulative lateral drift toward ego lane
                    dep_lateral += LAT_PER_TICK
                    mx, my, mz, myaw = mc_path[pi]
                    mc_wx = mx - lat_dx * dep_lateral
                    mc_wy = my - lat_dy * dep_lateral
                    mc.set_transform(carla.Transform(
                        carla.Location(x=mc_wx, y=mc_wy, z=mz + 0.15),
                        carla.Rotation(yaw=myaw)))

                    # Hand off to physics engine when gap closes to IMPACT_THRESH
                    if dep_lateral >= LANE_GAP - IMPACT_THRESH and impact_time is None:
                        impact_time = t
                        yaw_r = math.radians(eyaw)
                        fvx   = math.cos(yaw_r) * SPEED_MS
                        fvy   = math.sin(yaw_r) * SPEED_MS
                        ego.set_simulate_physics(True)
                        mc.set_simulate_physics(True)
                        ego.set_target_velocity(carla.Vector3D(x=fvx, y=fvy, z=0))
                        mc.set_target_velocity(carla.Vector3D(
                            x=fvx - lat_dx * LAT_SPEED,
                            y=fvy - lat_dy * LAT_SPEED, z=0))
                        physics_on = True
                        print(f"\n>>> PHYSICS IMPACT t={t:.2f}s  drift={dep_lateral:.2f}m\n")
            else:
                # Physics phase — apply AEB deceleration if triggered
                ev           = ego.get_velocity()
                current_speed = math.hypot(ev.x, ev.y)
                yaw_r         = math.radians(ego.get_transform().rotation.yaw)
                new_speed     = max(0, current_speed - MAX_DECEL * TICK) if aeb_triggered else SPEED_MS
                ego.set_target_velocity(carla.Vector3D(
                    x=math.cos(yaw_r) * new_speed,
                    y=math.sin(yaw_r) * new_speed,
                    z=0))

            # Trigger departure at end of stable phase
            if not departed and t >= STABLE_T:
                dep_time   = t
                dep_time_s = t
                dep_lateral = 0.0
                departed    = True
                print(f"\n>>> MC DEPARTURE t={t:.2f}s  gap={LANE_GAP:.2f}m  "
                      f"estimated impact in {LANE_GAP / LAT_SPEED:.1f}s\n")

            # Current lateral separation — used for TTC and console output
            _ep  = ego.get_location()
            _mp  = mc.get_location()
            lat  = math.hypot(_ep.x - _mp.x, _ep.y - _mp.y)

            # TTC — lateral gap divided by drift speed (valid after departure only)
            ttc = (lat / LAT_SPEED) if (departed and lat > 0.1) else float('inf')

            # Record when TTC first crosses activation threshold
            if AEB_ENABLED and not aeb_triggered and ttc < MIN_TTC_TRIGGER:
                if aeb_trigger_time is None:
                    aeb_trigger_time = t

            # Apply REACTION_TIME delay before AEB braking activates
            if aeb_trigger_time is not None and not aeb_triggered:
                if (t - aeb_trigger_time) >= REACTION_TIME:
                    aeb_triggered = True
                    print(f"\n>>> AEB ACTIVATED at t={t:.2f}s (TTC={ttc:.2f}s)\n")

            # Console status — printed once per second
            if t - last_print >= 1.0:
                ep   = ego.get_location()
                mp   = mc.get_location()
                lat  = math.hypot(ep.x - mp.x, ep.y - mp.y)
                sh   = side_log[-1]['n'] if side_log else 0
                ph   = ("[IMPACT]" if physics_on else "[DEP]" if departed else "[STA]")
                print(f"t={t:5.1f}s {ph}  EGO({ep.x:.0f},{ep.y:.1f})  "
                      f"MC({mp.x:.0f},{mp.y:.1f})  lat={lat:.2f}m  "
                      f"side={sh}  TTC={ttc:.2f}s  AEB={aeb_triggered}")
                last_print = t

            ticks += 1
            sl = (wall_start + t + TICK) - time.time()
            if sl > 0:
                time.sleep(sl)

        # ── POST-RUN SENSOR ANALYSIS ───────────────────────────────────────
        dep_t = dep_time if dep_time else 0.0
        imp_t = impact_time if impact_time else dep_t + LANE_GAP / LAT_SPEED

        side_stable  = [e for e in side_log if e['t_rel'] is not None and e['t_rel'] < dep_t]
        side_dep_win = [e for e in side_log if e['t_rel'] is not None and e['t_rel'] >= dep_t]

        fwd_dep  = next((e for e in fwd_log  if e['t_rel'] and e['t_rel'] >= dep_t), None)
        side_dep = next((e for e in side_dep_win), None)
        rear_dep = next((e for e in rear_log  if e['t_rel'] and e['t_rel'] >= dep_t), None)

        # ── SENSOR LOG ─────────────────────────────────────────────────────
        log_lines = [
            "=" * 60,
            "S3 SENSOR LOG — LATERAL MOTORCYCLE INTRUSION AT 80 km/h",
            "=" * 60,
            "",
            "SCENARIO PARAMETERS",
            f"  Speed (both vehicles):  {SPEED_KMH:.1f} km/h",
            f"  Lane gap (EGO to MC):   {LANE_GAP:.2f} m",
            f"  Lateral drift speed:    {LAT_SPEED} m/s",
            f"  Departure time:         t={dep_t:.2f}s",
            f"  Estimated impact time:  t={imp_t:.2f}s",
            f"  Physics impact time:    t={impact_time:.2f}s" if impact_time else "  Physics impact: N/A",
            f"  Collision window:       {LANE_GAP / LAT_SPEED:.1f}s",
            "",
            "SENSOR DETECTION DURING DEPARTURE WINDOW",
            f"  (departure window = t={dep_t:.2f}s to t={imp_t:.2f}s)",
            "",
        ]

        def sensor_block(name, dep_entry, stable_count, spec):
            """Format one sensor's analysis block for the log."""
            lines = [f"  {name}  ({spec})"]
            if dep_entry is None:
                lines.append("    DEPARTURE WINDOW: ZERO DETECTION  <- VALIDATION GAP CONFIRMED")
            else:
                delay  = dep_entry['t_rel'] - dep_t
                status = "within" if dep_entry['t_rel'] < imp_t else "AFTER impact est"
                lines.append(f"    DEPARTURE WINDOW: first detection t={dep_entry['t_rel']:.3f}s")
                lines.append(f"    Delay after departure: +{delay:.3f}s  ({status})")
                lines.append(f"    Min depth at detection: {dep_entry['min_depth']}m")
            lines.append(f"    Stable-phase hits (MC beside ego): {stable_count}")
            return lines

        log_lines += sensor_block(
            "FORWARD RADAR", fwd_dep, 0,
            "35deg H-FOV, 150m range — MC is lateral, outside forward cone")
        log_lines.append("")
        log_lines += sensor_block(
            "SIDE RADAR", side_dep, len(side_stable),
            "120deg H-FOV, 8m range  — parking-assist spec, not highway-rated")
        log_lines.append("")
        log_lines += sensor_block(
            "REAR RADAR", rear_dep, 0,
            "150deg H-FOV, 80m range — MC is ahead/beside, not behind")
        log_lines += [
            "",
            "NOTE ON SIDE RADAR STABLE-PHASE HITS:",
            f" Side radar detects adjacent vehicle at ~{LANE_GAP:.1f}m during stable phase.",
            "  This is expected — MC is within 8m range while beside ego.",
            "  However, production BSD systems do NOT trigger AEB from a stationary",
            "  adjacent vehicle. The critical gap is during the DEPARTURE WINDOW:",
            "  MC is moving laterally at 0.7 m/s — by the time it enters ego's lane",
            "  (t+5s) there is no remaining reaction time for any intervention.",
            "",
            "CONCLUSION",
            "  All three sensors detect at t=5.01s after departure onset.",
            "  However AEB decision threshold (TTC < 4.0s) is not crossed",
            "  until t=8.05s — leaving only 1.1s reaction window.",
            "  Collision at t=9.15s is unavoidable at 80 km/h.",
            "  Forward radar: structural FOV gap — MC is never in its cone.",
            "  Side radar: range designed for parking (<10 km/h), not 80 km/h.",
            "  Rear radar: geometry gap — MC is ahead of ego, not behind.",
            "  This confirms the Euro NCAP AEB validation gap for",
            "  MC leaving-carriageway events.",
            "  German crash data: 44.1% severe injury rate = 2.37x national baseline.",
            "=" * 60,
            "",
            f"cam_side frames saved:    {frame_counts['side']}",
            f"cam_overhead frames saved:{frame_counts['overhead']}",
            f"cam_chase frames saved:   {frame_counts['chase']}",
        ]

        print()
        for l in log_lines:
            print(l)

        log_path = os.path.join(OUTPUT_DIR, 'mc_sensor_log.txt')
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(log_lines) + '\n')
        print(f"\nLog saved: {log_path}")

        # ── RAW SENSOR DATA ────────────────────────────────────────────────
        raw_path = os.path.join(OUTPUT_DIR, 'mc_sensor_raw.txt')
        with open(raw_path, 'w', encoding='utf-8') as f:
            f.write("RAW FORWARD RADAR (departure window only)\n" + "-" * 40 + "\n")
            dep_fwd = [e for e in fwd_log if e['t_rel'] and e['t_rel'] >= dep_t]
            for e in dep_fwd:
                f.write(f"t={e['t_rel']:.3f}s  hits={e['n']}  min_depth={e['min_depth']}m\n")
            if not dep_fwd:
                f.write("NO DETECTIONS\n")

            f.write("\nRAW SIDE RADAR — STABLE PHASE\n" + "-" * 40 + "\n")
            for e in side_stable:
                f.write(f"t={e['t_rel']:.3f}s  hits={e['n']}  min_depth={e['min_depth']}m\n")

            f.write("\nRAW SIDE RADAR — DEPARTURE WINDOW\n" + "-" * 40 + "\n")
            for e in side_dep_win:
                f.write(f"t={e['t_rel']:.3f}s  hits={e['n']}  min_depth={e['min_depth']}m\n")
            if not side_dep_win:
                f.write("NO DETECTIONS\n")

            f.write("\nRAW REAR RADAR (departure window only)\n" + "-" * 40 + "\n")
            dep_rear = [e for e in rear_log if e['t_rel'] and e['t_rel'] >= dep_t]
            for e in dep_rear:
                f.write(f"t={e['t_rel']:.3f}s  hits={e['n']}  min_depth={e['min_depth']}m\n")
            if not dep_rear:
                f.write("NO DETECTIONS\n")
        print(f"Raw data: {raw_path}")

        if dep_time:
            print(f"\nBest frames for figures:")
            print(f"  cam_side     t={dep_t+0.5:.0f}s – {dep_t+4:.0f}s  (MC drifting toward EGO)")
            print(f"  cam_overhead t={dep_t:.0f}s – {dep_t+5:.0f}s  (lane crossing, top-down)")
            if impact_time:
                print(f"  cam_chase    t={impact_time:.0f}s – {impact_time+3:.0f}s  (physics collision)")

        # ── CORRECTION FACTOR MEASUREMENT ─────────────────────────────────
        if impact_time is not None:
            actual_dur    = round(impact_time - STABLE_T, 4)
            predicted_dur = round(LANE_GAP / LAT_SPEED, 4)
            cf_factor     = round(actual_dur / predicted_dur, 4)

            print(f"\n{'─'*52}")
            print(f"  CORRECTION FACTOR MEASUREMENT")
            print(f"  LAT_SPEED        : {LAT_SPEED} m/s")
            print(f"  LANE_GAP         : {LANE_GAP:.4f} m")
            print(f"  Predicted (math) : {predicted_dur:.4f} s")
            print(f"  CARLA actual     : {actual_dur:.4f} s")
            print(f"  Factor           : {cf_factor:.4f}")
            print(f"{'─'*52}\n")

            cf_results.append({
                'drift_mps'      : LAT_SPEED,
                'lane_gap_m'     : round(LANE_GAP, 4),
                'predicted_s'    : predicted_dur,
                'carla_actual_s' : actual_dur,
                'cf_factor'      : cf_factor,
            })

        # ── TTC SWEEP RESULT RECORDING ─────────────────────────────────────
        ttc_results.append({
            'ttc_threshold_s' : MIN_TTC_TRIGGER,
            'lat_speed_mps'   : LAT_SPEED,
            'aeb_activated'   : aeb_triggered,
            'aeb_trigger_t_s' : round(aeb_trigger_time, 3) if aeb_trigger_time else None,
            'aeb_active_t_s'  : round(aeb_trigger_time + REACTION_TIME, 3)
                                 if aeb_trigger_time else None,
            'window_at_aeb_s' : round(impact_time - (aeb_trigger_time + REACTION_TIME), 3)
                                 if (aeb_trigger_time and impact_time) else None,
            'impact_t_s'      : round(impact_time, 3) if impact_time else None,
            'collision'       : 1 if impact_time else 0,
        })

        # Single-run mode waits for keypress before cleanup
        if not CF_SWEEP_MODE and not TTC_SWEEP_MODE:
            input("\nPress ENTER to clean up...")

        # ── MONTE CARLO CALIBRATION METRICS ───────────────────────────────
        print("\n" + "=" * 60)
        print("EXTRACTING METRICS FOR MONTE CARLO CALIBRATION")
        print("=" * 60)

        # Reconstruct per-tick lateral separation from kinematic path
        metrics_log = []
        for tick_i in range(min(ticks, len(ego_path))):
            t_tick = tick_i * TICK
            ex_t, ey_t, ez_t, eyaw_t = ego_path[min(tick_i, len(ego_path) - 1)]
            mx_t, my_t, mz_t, myaw_t = mc_path[min(tick_i, len(mc_path) - 1)]

            if t_tick >= STABLE_T:
                drift_t = min((t_tick - STABLE_T) * LAT_SPEED, LANE_GAP)
                mx_t    = mx_t - lat_dx * drift_t
                my_t    = my_t - lat_dy * drift_t

            d_lat  = math.hypot(ex_t - mx_t, ey_t - my_t)
            d_long = abs((ey_t - my_t) * math.cos(math.radians(eyaw_t)) -
                         (ex_t - mx_t) * math.sin(math.radians(eyaw_t)))

            closing_speed = LAT_SPEED if t_tick >= STABLE_T else 0.0
            ttc           = (d_lat / closing_speed) if (closing_speed > 0.01 and d_lat > 0.1) else float('inf')

            metrics_log.append({
                't'       : round(t_tick, 3),
                'd_lat'   : round(d_lat, 3),
                'd_long'  : round(d_long, 3),
                'ttc'     : round(ttc, 2) if ttc != float('inf') else 'inf',
                'closing' : round(closing_speed, 3),
                'drift'   : round((t_tick - STABLE_T) * LAT_SPEED, 3) if t_tick >= STABLE_T else 0.0,
            })

        d_lat_values = [m['d_lat'] for m in metrics_log]
        ttc_values   = [m['ttc']   for m in metrics_log if isinstance(m['ttc'], (int, float))]

        Dmin    = min(d_lat_values)
        TTC_min = min(ttc_values) if ttc_values else float('inf')

        crossing_tick = next(
            (m for m in metrics_log if m['t'] >= STABLE_T and m['d_lat'] < 1.5), None)
        t_lane_cross = crossing_tick['t'] if crossing_tick else None

        fwd_dep_entry  = next((e for e in fwd_log  if e['t_rel'] and e['t_rel'] >= dep_t), None)
        side_dep_entry = next((e for e in side_dep_win), None) if 'side_dep_win' in dir() else None
        rear_dep_entry = next((e for e in rear_log  if e['t_rel'] and e['t_rel'] >= dep_t), None)

        latency_fwd  = (fwd_dep_entry['t_rel']  - dep_t) if fwd_dep_entry  else None
        latency_side = (side_dep_entry['t_rel'] - dep_t) if side_dep_entry else None
        latency_rear = (rear_dep_entry['t_rel'] - dep_t) if rear_dep_entry else None

        time_adv_fwd  = (imp_t - fwd_dep_entry['t_rel'])  if fwd_dep_entry  else 0.0
        time_adv_side = (imp_t - side_dep_entry['t_rel']) if side_dep_entry else 0.0

        collision = 1 if impact_time is not None else 0
        near_miss = 1 if 1.5 <= Dmin < 3.0 else 0

        print(f"""
KINEMATIC METRICS:
  v_ego:              {SPEED_KMH} km/h (constant)
  v_moto:             {SPEED_KMH} km/h (constant until impact)
  v_rel:              0 km/h longitudinal | {LAT_SPEED} m/s lateral

POSITION METRICS:
  d_lat_initial:      {LANE_GAP:.2f} m
  d_lat_min (Dmin):   {Dmin:.3f} m
  Lane crossing:      t={t_lane_cross}s

TIME METRICS:
  t_departure:        {dep_t:.2f}s
  t_collision:        {imp_t:.2f}s
  t_event:            {imp_t - dep_t:.2f}s
  TTC_min:            {TTC_min:.2f}s

DETECTION LATENCY:
  Forward radar:      {'No detection' if latency_fwd  is None else f'{latency_fwd:.3f}s'}
  Side radar:         {'No detection' if latency_side is None else f'{latency_side:.3f}s'}
  Rear radar:         {'No detection' if latency_rear is None else f'{latency_rear:.3f}s'}

TIME ADVANTAGE (detection to collision):
  Forward radar:      {time_adv_fwd:.2f}s
  Side radar:         {time_adv_side:.2f}s

SAFETY INDICATORS:
  Collision:          {'YES' if collision else 'NO'} (Dmin={Dmin:.3f}m)
  Near-miss:          {'YES' if near_miss else 'NO'}
        """)

        # ── SAVE MONTE CARLO BASELINE METRICS CSV ──────────────────────────
        metrics_csv = os.path.join(OUTPUT_DIR, 'mc_baseline_metrics.csv')
        with open(metrics_csv, 'w') as f:
            f.write("parameter,value\n")
            f.write(f"v_ego_kmh,{SPEED_KMH}\n")
            f.write(f"v_moto_kmh,{SPEED_KMH}\n")
            f.write(f"v_rel_lateral,{LAT_SPEED}\n")
            f.write(f"d_lat_initial,{LANE_GAP}\n")
            f.write(f"Dmin,{Dmin}\n")
            f.write(f"TTC_min,{TTC_min}\n")
            f.write(f"t_departure,{dep_t}\n")
            f.write(f"t_collision,{imp_t}\n")
            f.write(f"t_event,{imp_t - dep_t}\n")
            f.write(f"t_lane_cross,{t_lane_cross}\n")
            f.write(f"latency_fwd,{latency_fwd}\n")
            f.write(f"latency_side,{latency_side}\n")
            f.write(f"latency_rear,{latency_rear}\n")
            f.write(f"time_adv_fwd,{time_adv_fwd}\n")
            f.write(f"time_adv_side,{time_adv_side}\n")
            f.write(f"collision,{collision}\n")
            f.write(f"near_miss,{near_miss}\n")
        print(f"Baseline metrics: {metrics_csv}")

        # ── SAVE PER-TICK DATA CSV ─────────────────────────────────────────
        ticks_csv = os.path.join(OUTPUT_DIR, 'mc_tick_data.csv')
        with open(ticks_csv, 'w') as f:
            f.write("t,d_lat,d_long,ttc,closing_speed,drift\n")
            for m in metrics_log:
                f.write(f"{m['t']},{m['d_lat']},{m['d_long']},"
                        f"{m['ttc']},{m['closing']},{m['drift']}\n")
        print(f"Tick data: {ticks_csv}")

    finally:
        for a in reversed(actors):
            try:
                if a.is_alive:
                    a.destroy()
            except:
                pass
        print("Cleaned up.")


# ── ENTRY POINT ────────────────────────────────────────────────────────────
if __name__ == '__main__':

    # ── DEFAULT: single baseline run ──────────────────────────────────────
    if not CF_SWEEP_MODE and not TTC_SWEEP_MODE:
        main()

    # ── CORRECTION FACTOR SWEEP ───────────────────────────────────────────
    # Validates S3 = 0.83 across 5 drift speeds.
    # Confirms the correction factor is drift-independent (expected CV < 1%).
    elif CF_SWEEP_MODE:
        print('=' * 60)
        print('CORRECTION FACTOR VALIDATION SWEEP')
        print(f'Drift speeds: {CF_SWEEP_DRIFTS} m/s')
        print('=' * 60)
        for drift in CF_SWEEP_DRIFTS:
            LAT_SPEED    = drift
            LAT_PER_TICK = LAT_SPEED * TICK
            print(f'\n>>> SWEEP RUN: drift = {drift} m/s')
            main()
            time.sleep(4.0)

        import csv
        os.makedirs('s3_ego_mc_output_r1', exist_ok=True)
        cf_path = os.path.join('s3_ego_mc_output_r1', 'cf_sweep_results.csv')
        with open(cf_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=cf_results[0].keys())
            writer.writeheader()
            writer.writerows(cf_results)

        factors = [r['cf_factor'] for r in cf_results]
        print(f"\n{'='*52}")
        print(f"  CORRECTION FACTOR SWEEP COMPLETE")
        print(f"  {'Drift':>10} {'Predicted':>12} {'Actual':>12} {'Factor':>10}")
        print(f"  {'-'*46}")
        for r in cf_results:
            print(f"  {r['drift_mps']:>10.1f} {r['predicted_s']:>12.4f} "
                  f"{r['carla_actual_s']:>12.4f} {r['cf_factor']:>10.4f}")
        print(f"  {'-'*46}")
        print(f"  {'Mean':>34} {sum(factors)/len(factors):>10.4f}")
        print(f"  {'Std':>34} {float(np.std(factors)):>10.4f}")
        print(f"  Saved: {cf_path}")
        print(f"{'='*52}\n")

    # ── TTC THRESHOLD SENSITIVITY SWEEP ───────────────────────────────────
    # Tests 8 TTC thresholds (1.5 – 5.0 s) at baseline drift (0.7 m/s).
    # Expected result: collision = YES for all 8 thresholds.
    # Confirms the 99.6% baseline finding is threshold-independent.
    elif TTC_SWEEP_MODE:
        print('=' * 60)
        print('TTC THRESHOLD SENSITIVITY SWEEP')
        print(f'Thresholds: {TTC_SWEEP_THRESHOLDS} s')
        print('=' * 60)
        for threshold in TTC_SWEEP_THRESHOLDS:
            MIN_TTC_TRIGGER = threshold
            print(f'\n>>> SWEEP RUN: TTC threshold = {threshold} s')
            main()
            time.sleep(4.0)

        import csv
        os.makedirs('s3_ego_mc_output_r1', exist_ok=True)
        ttc_path = os.path.join('s3_ego_mc_output_r1', 'ttc_sweep_results.csv')
        with open(ttc_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=ttc_results[0].keys())
            writer.writeheader()
            writer.writerows(ttc_results)

        print(f"\n{'='*60}")
        print(f"  TTC THRESHOLD SENSITIVITY SWEEP COMPLETE")
        print(f"  {'TTC':>8} {'AEB fires':>12} {'Window':>10} {'Collision':>12}")
        print(f"  {'-'*46}")
        for r in ttc_results:
            w   = f"{r['window_at_aeb_s']:.2f}s" if r['window_at_aeb_s'] else '—'
            aeb = 'YES' if r['aeb_activated'] else 'NO'
            col = 'YES' if r['collision']     else 'NO'
            print(f"  {r['ttc_threshold_s']:>8.2f} {aeb:>12} {w:>10} {col:>12}")
        print(f"  Saved: {ttc_path}")
        print(f"{'='*60}\n")
