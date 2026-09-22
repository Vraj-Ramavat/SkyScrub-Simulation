#!/usr/bin/env python3

import csv
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SDF_FILE = ROOT / "models" / "skyscrub" / "model.sdf"
AIRFRAME_FILE = ROOT / "px4" / "22000_gz_skyscrub"
SCENARIO_FILE = ROOT / "config" / "flight_mass_scenarios_v0.2.csv"

# ---------------------------------------------------------------------
# Manufacturer-derived geometry / propulsion data
# ---------------------------------------------------------------------

WHEELBASE = 1.407                     # m, EFT E610P
ROTOR_RADIUS_FROM_CENTER = WHEELBASE / 2.0

X = ROTOR_RADIUS_FROM_CENTER * math.sqrt(3.0) / 2.0
Y = ROTOR_RADIUS_FROM_CENTER / 2.0

PROP_DIAMETER = 23.0 * 0.0254        # X6: 23 inch
PROP_RADIUS = PROP_DIAMETER / 2.0
PROP_MASS = 0.107                    # kg, complete prop assembly

X6_COMBO_MASS = 0.765                # kg / axis
FRAME_MASS = 5.920                   # kg

# Official X6 max speed = 6198 RPM
MAX_RPM = 6198.0
MAX_ROT_VEL = MAX_RPM * 2.0 * math.pi / 60.0

# Derived least-squares approximation from Hobbywing thrust/RPM table.
# Units: N / (rad/s)^2
MOTOR_CONSTANT = 0.000255

# ---------------------------------------------------------------------
# Hobbywing measured throttle/thrust data
# throttle fraction, thrust kg per rotor
# ---------------------------------------------------------------------

THRUST_TABLE = [
    (0.40, 2.645),
    (0.42, 2.895),
    (0.44, 3.175),
    (0.46, 3.415),
    (0.48, 3.685),
    (0.50, 3.975),
    (0.52, 4.225),
    (0.54, 4.575),
    (0.56, 4.745),
    (0.58, 5.160),
    (0.60, 5.570),
    (0.62, 5.845),
    (0.64, 6.110),
    (0.66, 6.435),
    (0.68, 6.875),
    (0.70, 7.165),
    (0.72, 7.595),
    (0.74, 8.055),
    (0.76, 8.320),
    (0.78, 8.815),
    (0.80, 9.170),
    (0.90, 10.285),
    (1.00, 11.315),
]


def interpolate_hover_throttle(thrust_kg):
    if thrust_kg <= THRUST_TABLE[0][1]:
        return THRUST_TABLE[0][0]

    for i in range(len(THRUST_TABLE) - 1):
        t0, f0 = THRUST_TABLE[i]
        t1, f1 = THRUST_TABLE[i + 1]

        if f0 <= thrust_kg <= f1:
            ratio = (thrust_kg - f0) / (f1 - f0)
            return t0 + ratio * (t1 - t0)

    return 1.0


def read_scenarios():
    scenarios = {}

    with SCENARIO_FILE.open() as f:
        reader = csv.DictReader(f)

        for row in reader:
            scenarios[row["scenario"].upper()] = {
                "water": float(row["water_kg"]),
                "total": float(row["total_mass_kg"]),
            }

    return scenarios


def set_text(element, value):
    if element is None:
        raise RuntimeError("Required SDF element was not found")

    element.text = str(value)


def modify_sdf(total_mass):
    tree = ET.parse(SDF_FILE)
    root = tree.getroot()

    model = root.find("model")

    if model is None:
        raise RuntimeError("No <model> element found")

    base = model.find("./link[@name='base_link']")

    if base is None:
        raise RuntimeError("base_link not found")

    # -------------------------------------------------------------
    # Rotor / propeller mass
    #
    # The rest of each 765 g X6 propulsion combo remains represented
    # in the base-link inertia model.
    # -------------------------------------------------------------

    rotor_total_mass = 6.0 * PROP_MASS
    base_mass = total_mass - rotor_total_mass

    set_text(
        base.find("./inertial/mass"),
        f"{base_mass:.6f}"
    )

    # -------------------------------------------------------------
    # Provisional inertia approximation
    #
    # Frame:
    #   approximated as six radial members.
    #
    # Stationary propulsion mass:
    #   motor + ESC mass positioned near rotor radius.
    #
    # Central equipment:
    #   approximated as a compact central cylinder.
    #
    # This is a flight-dynamics planning model, not CAD-derived inertia.
    # -------------------------------------------------------------

    stationary_x6_mass = 6.0 * (X6_COMBO_MASS - PROP_MASS)

    central_mass = (
        total_mass
        - FRAME_MASS
        - 6.0 * X6_COMBO_MASS
    )

    if central_mass <= 0:
        raise RuntimeError("Calculated central mass is invalid")

    central_radius = 0.22
    central_height = 0.30

    frame_izz = (
        FRAME_MASS
        * ROTOR_RADIUS_FROM_CENTER ** 2
        / 3.0
    )

    frame_ixx = frame_izz / 2.0

    propulsion_izz = (
        stationary_x6_mass
        * ROTOR_RADIUS_FROM_CENTER ** 2
    )

    propulsion_ixx = propulsion_izz / 2.0

    central_izz = (
        0.5
        * central_mass
        * central_radius ** 2
    )

    central_ixx = (
        central_mass
        * (
            3.0 * central_radius ** 2
            + central_height ** 2
        )
        / 12.0
    )

    base_ixx = frame_ixx + propulsion_ixx + central_ixx
    base_iyy = base_ixx
    base_izz = frame_izz + propulsion_izz + central_izz

    inertia = base.find("./inertial/inertia")

    set_text(inertia.find("ixx"), f"{base_ixx:.6f}")
    set_text(inertia.find("iyy"), f"{base_iyy:.6f}")
    set_text(inertia.find("izz"), f"{base_izz:.6f}")

    # -------------------------------------------------------------
    # E610P 1407 mm motor geometry
    # -------------------------------------------------------------

    arm_midpoints = [
        (0.0, ROTOR_RADIUS_FROM_CENTER / 2.0, 1.57079633),
        (0.0, -ROTOR_RADIUS_FROM_CENTER / 2.0, -1.57079633),
        (X / 2.0, -Y / 2.0, -0.52359878),
        (-X / 2.0, Y / 2.0, 2.61799388),
        (X / 2.0, Y / 2.0, 0.52359878),
        (-X / 2.0, -Y / 2.0, -2.61799388),
    ]

    rotor_positions = [
        (0.0, ROTOR_RADIUS_FROM_CENTER),
        (0.0, -ROTOR_RADIUS_FROM_CENTER),
        (X, -Y),
        (-X, Y),
        (X, Y),
        (-X, -Y),
    ]

    for i, (mx, my, yaw) in enumerate(arm_midpoints):
        for kind in ("visual", "collision"):
            item = base.find(
                f"./{kind}[@name='arm_{i}_{kind}']"
            )

            if item is None:
                raise RuntimeError(
                    f"arm_{i}_{kind} was not found"
                )

            pose = item.find("pose")

            pose.text = (
                f"{mx:.6f} {my:.6f} "
                f"0.020000 0 0 {yaw:.8f}"
            )

            size = item.find("./geometry/box/size")
            values = size.text.split()
            values[0] = f"{ROTOR_RADIUS_FROM_CENTER:.6f}"
            size.text = " ".join(values)

    # -------------------------------------------------------------
    # Rotor links
    # -------------------------------------------------------------

    # Axisymmetric finite-thickness approximation for the rotating
    # 23-inch propeller. Using finite thickness avoids the degenerate
    # thin-disc inertia condition rejected by Gazebo.
    prop_thickness = 0.008

    prop_ixx = (
        PROP_MASS
        * (
            3.0 * PROP_RADIUS ** 2
            + prop_thickness ** 2
        )
        / 12.0
    )

    prop_izz = (
        0.5
        * PROP_MASS
        * PROP_RADIUS ** 2
    )

    for i, (rx, ry) in enumerate(rotor_positions):
        rotor = model.find(f"./link[@name='rotor_{i}']")

        if rotor is None:
            raise RuntimeError(f"rotor_{i} not found")

        pose = rotor.find("pose")

        pose.text = (
            f"{rx:.6f} {ry:.6f} "
            "0.100000 0 0 0"
        )

        set_text(
            rotor.find("./inertial/mass"),
            f"{PROP_MASS:.6f}"
        )

        rotor_inertia = rotor.find("./inertial/inertia")

        set_text(
            rotor_inertia.find("ixx"),
            f"{prop_ixx:.8f}"
        )

        set_text(
            rotor_inertia.find("iyy"),
            f"{prop_ixx:.8f}"
        )

        set_text(
            rotor_inertia.find("izz"),
            f"{prop_izz:.8f}"
        )

        radius = rotor.find(
            f"./visual[@name='rotor_{i}_visual']"
            "/geometry/cylinder/radius"
        )

        set_text(radius, f"{PROP_RADIUS:.6f}")

    # -------------------------------------------------------------
    # X6 motor model calibration
    # -------------------------------------------------------------

    motor_plugins = []

    for plugin in model.findall("plugin"):
        filename = plugin.attrib.get("filename", "")

        if "multicopter-motor-model" in filename:
            motor_plugins.append(plugin)

    if len(motor_plugins) != 6:
        raise RuntimeError(
            f"Expected 6 motor plugins, found {len(motor_plugins)}"
        )

    for plugin in motor_plugins:
        set_text(
            plugin.find("maxRotVelocity"),
            f"{MAX_ROT_VEL:.6f}"
        )

        set_text(
            plugin.find("motorConstant"),
            f"{MOTOR_CONSTANT:.9f}"
        )

    ET.indent(tree, space="  ")

    tree.write(
        SDF_FILE,
        encoding="UTF-8",
        xml_declaration=True,
    )

    return {
        "base_mass": base_mass,
        "base_ixx": base_ixx,
        "base_iyy": base_iyy,
        "base_izz": base_izz,
        "central_mass": central_mass,
        "prop_izz": prop_izz,
        "prop_ixx": prop_ixx,
    }


def replace_parameter(text, name, value):
    pattern = rf"(^\s*param set-default {re.escape(name)}\s+).*$"

    new_text, count = re.subn(
        pattern,
        rf"\g<1>{value}",
        text,
        flags=re.MULTILINE,
    )

    if count == 0:
        raise RuntimeError(
            f"Parameter {name} was not found in airframe file"
        )

    return new_text


def modify_airframe(total_mass):
    text = AIRFRAME_FILE.read_text()

    positions = {
        "CA_ROTOR0_PX": 0.0,
        "CA_ROTOR0_PY": -ROTOR_RADIUS_FROM_CENTER,

        "CA_ROTOR1_PX": 0.0,
        "CA_ROTOR1_PY": ROTOR_RADIUS_FROM_CENTER,

        "CA_ROTOR2_PX": X,
        "CA_ROTOR2_PY": Y,

        "CA_ROTOR3_PX": -X,
        "CA_ROTOR3_PY": -Y,

        "CA_ROTOR4_PX": X,
        "CA_ROTOR4_PY": -Y,

        "CA_ROTOR5_PX": -X,
        "CA_ROTOR5_PY": Y,
    }

    for name, value in positions.items():
        text = replace_parameter(
            text,
            name,
            f"{value:.6f}",
        )

    # Gazebo ESC output uses natural output units.
    # Match the simulated X6 upper rotational-speed command.
    for i in range(1, 7):
        text = replace_parameter(
            text,
            f"SIM_GZ_EC_MAX{i}",
            "649",
        )

    # Gazebo motor plugin is driven in angular-velocity natural units.
    # Therefore MPC_THR_HOVER must start from the simulated rotor-speed
    # requirement rather than the physical ESC throttle percentage.
    thrust_newton_per_motor = (
        total_mass * 9.80665 / 6.0
    )

    required_omega = math.sqrt(
        thrust_newton_per_motor / MOTOR_CONSTANT
    )

    esc_min = 120.0
    esc_max = 649.0

    hover = (
        required_omega - esc_min
    ) / (
        esc_max - esc_min
    )

    hover = max(0.05, min(0.95, hover))

    text = replace_parameter(
        text,
        "MPC_THR_HOVER",
        f"{hover:.3f}",
    )

    AIRFRAME_FILE.write_text(text)

    return hover


def main():
    scenarios = read_scenarios()

    scenario = (
        sys.argv[1].upper()
        if len(sys.argv) > 1
        else "FULL"
    )

    if scenario not in scenarios:
        print("Valid scenarios:")
        for key in scenarios:
            print(f"  {key}")
        raise SystemExit(1)

    water = scenarios[scenario]["water"]
    total = scenarios[scenario]["total"]

    result = modify_sdf(total)
    hover = modify_airframe(total)

    print("=" * 72)
    print("SKYSCRUB V0.2 SCENARIO BUILT")
    print("=" * 72)

    print(f"Scenario                    : {scenario}")
    print(f"Water mass                  : {water:.3f} kg")
    print(f"Total vehicle mass          : {total:.3f} kg")
    print()

    print(f"E610P wheelbase             : {WHEELBASE:.4f} m")
    print(
        f"Motor radius                : "
        f"{ROTOR_RADIUS_FROM_CENTER:.4f} m"
    )

    print(
        f"Diagonal rotor X            : "
        f"{X:.6f} m"
    )

    print(
        f"Diagonal rotor Y            : "
        f"{Y:.6f} m"
    )

    print()

    print(
        f"Propeller diameter          : "
        f"{PROP_DIAMETER:.4f} m (23 inch)"
    )

    print(
        f"Propeller mass ×6           : "
        f"{PROP_MASS:.3f} kg each"
    )

    print(
        f"Base-link mass              : "
        f"{result['base_mass']:.3f} kg"
    )

    print()

    print("Base-link inertia approximation:")
    print(f"  Ixx                       : {result['base_ixx']:.4f}")
    print(f"  Iyy                       : {result['base_iyy']:.4f}")
    print(f"  Izz                       : {result['base_izz']:.4f}")

    print()

    print(
        f"X6 max rotational velocity : "
        f"{MAX_ROT_VEL:.2f} rad/s"
    )

    print(
        f"X6 motor constant           : "
        f"{MOTOR_CONSTANT:.9f}"
    )

    print(
        f"Initial PX4 hover estimate  : "
        f"{hover:.3f}"
    )

    print()
    print(
        "NOTE: inertia is a documented planning approximation, "
        "not a CAD-derived final value."
    )

    print("=" * 72)


if __name__ == "__main__":
    main()
