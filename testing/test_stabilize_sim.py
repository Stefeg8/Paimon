import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.01  # time step
T = 10     # total time
N = int(T / dt)
time = np.linspace(0, T, N)

# Physical parameters
mass = 1.0  # kg
gravity = 9.81  # m/s^2
target_z = -1.6  # meters (NED frame, down is positive)

# PID gains
kp_pos = 1.0
kd_pos = 0.1

kp_vel = 5.0
kd_vel = 0.1

# State variables
z = 0.0
vz = 0.0
az = 0.0

# Logging
z_log = []
thrust_log = []

for t in time:
    # --- Altitude Controller ---
    # Position control (z is in NED, so lower altitude = more negative)
    z_error = target_z - z
    vz_setpoint = kp_pos * z_error

    # Velocity control
    vz_error = vz_setpoint - vz
    az_cmd = kp_vel * vz_error

    # Gravity compensation and thrust calculation
    total_acc = az_cmd + gravity
    thrust = mass * total_acc / gravity  # Normalize thrust to 1.0 = hover

    # Clamp thrust
    thrust = max(0.0, min(thrust, 2.0))  # assume motors can give 2x gravity thrust

    # --- Physics Simulation ---
    az = (thrust * gravity - gravity)  # net acceleration
    vz += az * dt
    z += vz * dt

    # Log data
    z_log.append(z)
    thrust_log.append(thrust)

# Plotting
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(time, z_log)
plt.title("Simulated Altitude (NED Z)")
plt.ylabel("Z (m)")

plt.subplot(2, 1, 2)
plt.plot(time, thrust_log)
plt.title("Thrust Command")
plt.ylabel("Thrust (0–2)")
plt.xlabel("Time (s)")

plt.tight_layout()
plt.show()
