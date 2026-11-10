from matplotlib import pyplot as plt
import numpy as np
import json

with open("parameters.json", "r",encoding="utf-8") as f:
    params = json.load(f)

    vehicle = params["vehicle"]
    drivetrain = params["drivetrain"]
    motor = params["motor"]
    angle = params["environment"]["angle_deg"]


    mass, length, height, width, drag_coeff, frontal_area, rolling_resistance_coeff, air_density = vehicle.values()
    motor_count, drive_wheel_radius, gear_ratio, gear_mesh_loss, gear_bearing_friction = drivetrain.values()
    kt, ra, friction_torque, nom_voltage, nom_current = motor.values()

T = 20          
dt = 0.01       
N = int(T/dt)   
g = 9.81

alpha = angle*np.pi/180

#Vektorer
pos = np.zeros(N+1)
a = np.zeros(N+1)
v = np.zeros(N+1)
U = np.zeros(N+1)
I = np.zeros(N+1)


for i in range(0,N):
    
    s = np.sign(v[i]) or 1

    j = (nom_voltage/2 - kt * gear_ratio*v[i]/drive_wheel_radius) / ra
    j = min(j, nom_current)
    I[i] = j
    U[i] = ra * j + kt * v[i] / (drive_wheel_radius) * gear_ratio


    motor_torque = kt * j
        
    net_torque = (((motor_torque - friction_torque) * gear_ratio) * (1 - gear_mesh_loss) - gear_bearing_friction * s) * motor_count

    F_drive = net_torque / drive_wheel_radius
    F_drag = 0.5 * drag_coeff * frontal_area * air_density * v[i] ** 2 *np.sign(v[i])
    F_roll = rolling_resistance_coeff * mass * g * np.cos(alpha) * s
    F_grav = mass * g * np.sin(alpha)
    a[i] = (F_drive - F_drag - F_roll - F_grav)/mass
    v[i+1] = v[i] + a[i] * dt

print(v[N])