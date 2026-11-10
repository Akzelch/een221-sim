from matplotlib import pyplot as plt
import numpy as np
import json

with open("parameters.json", "r",encoding="utf-8") as f:
    params = json.load(f)

vehicle = params["vehicle"]
drivetrain = params["drivetrain"]
motor = params["motor"]

T = 20          
dt = 0.01       
N = int(T/dt)   
g = 9.82
angle = 0
alpha = angle*np.pi/180

#Vektorer
pos = np.zeros(N+1)
a = np.zeros(N+1)
v = np.zeros(N+1)
U = np.zeros(N+1)
I = np.zeros(N+1)

for i in range(0,N):
    

    motor_torque = motor["kt"] * (motor["nom_voltage"]/2 - motor["kt"]*drivetrain["gear_ratio"]*v[i]/drivetrain["drive_wheel_radius"])/motor["ra"]
    I[i] = motor_torque / motor["kt"]
    net_torque = (((motor_torque - motor["friction_torque"]) * drivetrain["gear_ratio"]) * (1 - drivetrain["gear_mesh_loss"]) - drivetrain["gear_bearing_friction"]) * 4

    F_drive = net_torque / drivetrain["drive_wheel_radius"]
    F_drag = 0.5 * vehicle["drag_coeff"] * vehicle["frontal_area"] * vehicle["air_density"] * v[i] ** 2 
    F_roll = vehicle["rolling_resistance_coeff"] * vehicle["mass"] * g * np.cos(alpha)
    F_grav = vehicle["mass"] * g * np.sin(alpha)
    #TODO inkorrekt hantering av bakåtgående fordon. np.sign(v) or 1 behöver läggas till i vissa fall
    a = (F_drive - F_drag - F_roll - F_grav)/vehicle["mass"]
