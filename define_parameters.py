def create_params():
    import json

    with open("parameters.json", "r",encoding="utf-8") as f:
        params = json.load(f)

    vehicle = params["vehicle"]
    drivetrain = params["drivetrain"]
    motor = params["motor"]


    mass, length, height, width, drag_coeff, frontal_area, rolling_resistance_coeff, air_density = vehicle.values()
    motor_count, drive_wheel_radius, gear_ratio, gear_mesh_loss, gear_bearing_friction = drivetrain.values()
    k_t, ra, friction_torque, nom_voltage, nom_current = motor.values()