import json
with open("parameters.json", "w", encoding="utf-8") as f:
    parameters = {
  "vehicle": { "mass_kg": 750, "length_m": 3.28, "height_m": 1.28, "width_m": 0.6,
               "drag_coeff": 0.3, "frontal_area_m2": 0.768,
               "rolling_resistance_coeff": 0.17, "air_density_kg_m3": 1.225 },
  "drivetrain": { "motor_count": 4, "drive_wheel_radius_m": 0.12, "gear_ratio": 1.2,
                  "gear_mesh_loss": 0.01, "gear_bearing_friction_nm": 0.54 },
  "motor": { "kt_vs": 1.542, "ra_ohm": 0.083, "friction_torque_nm": 1.6818,
             "nom_voltage_v": 264.8, "nom_current_a": 65.9 }
}
    json.dump(parameters, f, ensure_ascii=False, indent=4)

    print("success")