import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

"VEHICLE"
# Published/nominal values for a 2012 Ford Focus SEL sedan.
mass = 1331                     # kg
engine_power = 119300           # W, approximately 160 hp
max_torque = 198                # N*m, approximately 146 lb-ft
peak_torque_rpm = 4450          # rpm
peak_power_rpm = 6500           # rpm
drag_coefficient = 0.295
frontal_area = 2.20             # m^2, model estimate
front_weight_fraction = 0.60    # model estimate for FWD traction

"TIRES"
# Factory tire size: 215/55R16.
tire_width = 215                # mm
aspect_ratio = 0.55
rim_diameter = 16               # in

# Convert tire dimensions into nominal wheel radius.
sidewall = tire_width * aspect_ratio
tire_diameter = rim_diameter * 25.4 + 2 * sidewall
wheel_radius = tire_diameter / 2000

"TRANSMISSION"
# Six-speed PowerShift transmission and final-drive ratios.
gear_ratios = {1: 3.917, 2: 2.429, 3: 1.436, 4: 1.021, 5: 0.867, 6: 0.702}
final_drive = {1: 3.850, 2: 3.850, 3: 4.278, 4: 4.278, 5: 3.850, 6: 3.850}

drivetrain_efficiency = 0.90    # model assumption
idle_rpm = 850
redline_rpm = 7000

"ENGINE"
# Published peak torque/power values anchor the estimated torque curve.
# Intermediate torque values are model assumptions.
rpm_points = np.array([800, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4450, 5000, 5500, 6000, 6500, 7000])
torque_points = np.array([95, 120, 150, 170, 180, 188, 193, 197, 198, 195, 190, 184, 175, 155])

def engine_torque(rpm):
    # Estimate torque between known RPM points using linear interpolation.
    return np.interp(rpm, rpm_points, torque_points)

def engine_rpm(speed, gear):
    # Convert vehicle speed into wheel RPM, then engine RPM through gearing.
    overall_ratio = gear_ratios[gear] * final_drive[gear]
    wheel_rpm = speed / (2 * np.pi * wheel_radius) * 60
    rpm = wheel_rpm * overall_ratio

    # Assume clutch slip prevents the engine dropping below idle in first gear.
    if gear == 1 and rpm < idle_rpm:
        rpm = idle_rpm

    return rpm

"ENVIRONMENT"
air_density = 1.225             # kg/m^3
gravity = 9.81                  # m/s^2

"TRACK"
# Custom closed circuit made from straights and constant-radius corners.
# At a 1 m step size, this produces 2,580 simulation points per condition.
dx = 1.0                        # m

track_segments = [
    ("Start Straight", 420, "straight", np.inf),
    ("Turn 1", 120, "corner", 55),
    ("Short Straight", 220, "straight", np.inf),
    ("Turn 2", 160, "corner", 80),
    ("Back Straight", 500, "straight", np.inf),
    ("Turn 3", 140, "corner", 45),
    ("Mid Straight", 260, "straight", np.inf),
    ("Turn 4", 200, "corner", 100),
    ("Final Straight", 380, "straight", np.inf),
    ("Turn 5", 180, "corner", 65)
]

distance, radius, segment_name, segment_type = [], [], [], []
position = 0.0

# Break every track segment into individual 1 m simulation points.
for name, length, kind, turn_radius in track_segments:
    points = int(round(length / dx))

    for _ in range(points):
        distance.append(position)
        radius.append(turn_radius)
        segment_name.append(name)
        segment_type.append(kind)
        position += dx

# Convert lists to NumPy arrays for calculations.
distance = np.array(distance)
radius = np.array(radius)
segment_name = np.array(segment_name)
segment_type = np.array(segment_type)

track_length = len(distance) * dx

"RESISTANCE"
def resistive_force(speed, vehicle_mass, rolling_coefficient):
    # Aerodynamic drag increases with the square of speed.
    drag = 0.5 * air_density * drag_coefficient * frontal_area * speed**2

    # Rolling resistance is approximated as proportional to vehicle weight.
    rolling = rolling_coefficient * vehicle_mass * gravity

    return drag + rolling

"ACCELERATION"
def best_drive_force(speed, mu, vehicle_mass, local_radius):
    # Determine how much tire grip is already being used for cornering.
    lateral_acceleration = speed**2 / local_radius if np.isfinite(local_radius) else 0.0
    grip_limit = mu * gravity

    # Friction-circle model: cornering reduces grip available for acceleration.
    if lateral_acceleration >= grip_limit:
        grip_fraction = 0.0
    else:
        grip_fraction = np.sqrt(1 - (lateral_acceleration / grip_limit)**2)

    # FWD vehicle: front tires limit how much drive force can reach the road.
    tire_force_limit = mu * front_weight_fraction * vehicle_mass * gravity * grip_fraction

    best_force = 0.0
    best_gear = 1

    # Test every usable gear and choose the one producing the greatest wheel force.
    for gear in gear_ratios:
        rpm = engine_rpm(speed, gear)

        # Ignore gears that place the engine outside its usable RPM range.
        if gear != 1 and rpm < idle_rpm:
            continue
        if rpm > redline_rpm:
            continue

        torque = engine_torque(rpm)
        overall_ratio = gear_ratios[gear] * final_drive[gear]

        # Convert engine torque through gearing into force at the road.
        wheel_force = torque * overall_ratio * drivetrain_efficiency / wheel_radius

        if wheel_force > best_force:
            best_force = wheel_force
            best_gear = gear

    # Actual drive force cannot exceed available tire traction.
    return min(best_force, tire_force_limit), best_gear

def acceleration_limit(speed, mu, vehicle_mass, rolling_coefficient, local_radius):
    # Calculate the maximum usable driving force.
    drive_force, _ = best_drive_force(speed, mu, vehicle_mass, local_radius)

    # Newton's second law: acceleration = net force / mass.
    net_force = drive_force - resistive_force(speed, vehicle_mass, rolling_coefficient)
    return net_force / vehicle_mass

"BRAKING"
def braking_limit(speed, mu, vehicle_mass, rolling_coefficient, local_radius):
    # Calculate lateral acceleration already required by the corner.
    lateral_acceleration = speed**2 / local_radius if np.isfinite(local_radius) else 0.0
    grip_limit = mu * gravity

    # Remaining longitudinal tire grip is available for braking.
    longitudinal_grip = np.sqrt(max(grip_limit**2 - lateral_acceleration**2, 0.0))

    # Drag and rolling resistance also contribute to slowing the vehicle.
    resistance_deceleration = resistive_force(speed, vehicle_mass, rolling_coefficient) / vehicle_mass

    return longitudinal_grip + resistance_deceleration

"SIMULATION"
def simulate_condition(mu, vehicle_mass, rolling_coefficient):
    # Numerical ceiling; actual vehicle speed should remain below this naturally.
    speed_cap = 220 / 3.6

    # Begin with the maximum allowable speed at every track point.
    speed_limit = np.full(len(distance), speed_cap)
    corner_points = np.isfinite(radius)

    # Cornering limit from centripetal acceleration:
    # v^2/r = mu*g  ->  v = sqrt(mu*g*r)
    speed_limit[corner_points] = np.minimum(np.sqrt(mu * gravity * radius[corner_points]), speed_cap)

    speed = speed_limit.copy()

    # Repeated forward and backward passes solve the closed-lap speed profile.
    for _ in range(100):
        previous_speed = speed.copy()

        # Forward pass: determine how quickly the vehicle can accelerate.
        for i in range(len(speed)):
            next_i = (i + 1) % len(speed)

            acceleration = acceleration_limit(speed[i], mu, vehicle_mass, rolling_coefficient, radius[i])

            # Kinematics: v^2 = u^2 + 2as
            possible_speed = np.sqrt(max(speed[i]**2 + 2 * acceleration * dx, 0.0))
            speed[next_i] = min(speed[next_i], possible_speed)

        # Backward pass: determine how early the vehicle must brake for corners.
        for i in range(len(speed) - 1, -1, -1):
            previous_i = (i - 1) % len(speed)

            deceleration = braking_limit(speed[i], mu, vehicle_mass, rolling_coefficient, radius[previous_i])

            # Maximum previous speed that still allows braking to the next point.
            possible_speed = np.sqrt(max(speed[i]**2 + 2 * deceleration * dx, 0.0))
            speed[previous_i] = min(speed[previous_i], possible_speed)

        # Stop once repeated passes no longer significantly change the solution.
        if np.max(np.abs(speed - previous_speed)) < 0.0001:
            break

    # Calculate lap time using the average speed across each 1 m section.
    next_speed = np.roll(speed, -1)
    average_speed = (speed + next_speed) / 2
    lap_time = np.sum(dx / np.maximum(average_speed, 0.1))

    # Rearrange v^2 = u^2 + 2as to recover longitudinal acceleration.
    longitudinal_acceleration = (next_speed**2 - speed**2) / (2 * dx)

    return {"speed": speed, "acceleration": longitudinal_acceleration, "lap_time": lap_time}

"DRIVING CONDITIONS"
# Friction and rolling-resistance values are model assumptions used to compare conditions.
conditions = {
    "Dry": {"mu": 0.90, "mass": mass, "rolling": 0.012},
    "Damp": {"mu": 0.75, "mass": mass, "rolling": 0.013},
    "Wet": {"mu": 0.60, "mass": mass, "rolling": 0.014}
}

results = {}

# Run a complete lap simulation for every driving condition.
for name, values in conditions.items():
    results[name] = simulate_condition(values["mu"], values["mass"], values["rolling"])

"ANALYSIS"
# Convert the dry-condition speed profile from m/s to km/h.
dry_speed_kmh = results["Dry"]["speed"] * 3.6

# Determine minimum speed reached through every corner.
corner_names = [name for name, _, kind, _ in track_segments if kind == "corner"]
corner_minimums = {name: np.min(dry_speed_kmh[segment_name == name]) for name in corner_names}

# Lowest-speed corner is treated as the primary speed-limiting section.
limiting_corner = min(corner_minimums, key=corner_minimums.get)
limiting_speed = corner_minimums[limiting_corner]

# Determine maximum speed reached on the straight sections.
straight_mask = segment_type == "straight"
max_straight_speed = np.max(dry_speed_kmh[straight_mask])

"RESULTS"
print("SIMULATION")
print("Track length:", round(track_length / 1000, 2), "km")
print("Points per condition:", len(distance))
print("Total points evaluated:", len(distance) * len(conditions))

print("LAP TIMES")
for name in conditions:
    print(name + ":", round(results[name]["lap_time"], 2), "s")

print("DRY PERFORMANCE")
print("Maximum speed:", round(max_straight_speed, 1), "km/h")
print("Speed-limiting section:", limiting_corner, "-", round(limiting_speed, 1), "km/h")

"OUTPUT"
# Automatically create a figures folder beside main.py.
project_folder = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()
figure_folder = project_folder / "figures"
figure_folder.mkdir(exist_ok=True)

"SPEED PLOT"
# Compare velocity around the track for all three road conditions.
plt.figure(figsize=(10, 5))

for name in conditions:
    plt.plot(distance, results[name]["speed"] * 3.6, label=name)

plt.xlabel("Distance around track (m)")
plt.ylabel("Speed (km/h)")
plt.title("2012 Ford Focus SEL - Lap Speed Profile")
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig(figure_folder / "speed_profile.png", dpi=300)
plt.show()

"ACCELERATION PLOT"
# Positive values represent acceleration; negative values represent braking.
plt.figure(figsize=(10, 5))
plt.plot(distance, results["Dry"]["acceleration"])
plt.axhline(0)
plt.xlabel("Distance around track (m)")
plt.ylabel("Longitudinal acceleration (m/s²)")
plt.title("2012 Ford Focus SEL - Dry Lap Acceleration Profile")
plt.grid()
plt.tight_layout()
plt.savefig(figure_folder / "acceleration_profile.png", dpi=300)
plt.show()