#!/usr/bin/env python3

import csv
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

STATIC_BUILDER = ROOT / "scripts" / "build_v02_scenario.py"
SDF_FILE = ROOT / "models" / "skyscrub" / "model.sdf"
AIRFRAME_FILE = ROOT / "px4" / "22000_gz_skyscrub"
SCENARIO_FILE = ROOT / "config" / "flight_mass_scenarios_v0.2.csv"

PACKET_COUNT = 32
PACKET_MASS = 0.250

# Position of water centroid in the current planning model.
# This is intentionally centred so packet removal does not introduce
# artificial roll / pitch imbalance.
WATER_X = 0.0
WATER_Y = 0.0
WATER_Z = 0.300

# Small equivalent sphere inertia for each invisible packet.
PACKET_RADIUS = 0.020

MOTOR_CONSTANT = 0.000255
ESC_MIN = 120.0
ESC_MAX = 649.0
GRAVITY = 9.80665


def scenario_mass(name):
    with SCENARIO_FILE.open() as f:
        for row in csv.DictReader(f):
            if row["scenario"].upper() == name.upper():
                return float(row["total_mass_kg"])

    raise RuntimeError(f"Scenario {name} not found")


def replace_parameter(text, name, value):
    pattern = rf"(^\s*param set-default {re.escape(name)}\s+).*$"

    new_text, count = re.subn(
        pattern,
        rf"\g<1>{value}",
        text,
        flags=re.MULTILINE,
    )

    if count == 0:
        raise RuntimeError(f"{name} not found in airframe")

    return new_text


def main():

    dry_mass = scenario_mass("EMPTY")
    full_mass = scenario_mass("FULL")

    expected_water = full_mass - dry_mass

    if abs(expected_water - PACKET_COUNT * PACKET_MASS) > 1e-6:
        raise RuntimeError(
            "Water packet mass does not match FULL minus EMPTY mass"
        )

    # Generate the validated EMPTY vehicle first.
    # This gives us the dry base-link mass and dry inertia.
    subprocess.run(
        [sys.executable, str(STATIC_BUILDER), "EMPTY"],
        check=True,
    )

    tree = ET.parse(SDF_FILE)
    root = tree.getroot()

    model = root.find("model")

    if model is None:
        raise RuntimeError("Main SkyScrub model not found")

    # Remove old water packets if script is run more than once.
    for child in list(model.findall("model")):
        if child.attrib.get("name", "").startswith("water_packet_"):
            model.remove(child)

    for plugin in list(model.findall("plugin")):
        detach = plugin.find("detach_topic")

        if (
            detach is not None
            and detach.text
            and detach.text.startswith("/skyscrub/water/detach/")
        ):
            model.remove(plugin)

    packet_inertia = (
        (2.0 / 5.0)
        * PACKET_MASS
        * PACKET_RADIUS ** 2
    )

    # Add 32 independent 250 g water packet models.
    for i in range(PACKET_COUNT):

        packet_name = f"water_packet_{i:02d}"

        packet = ET.SubElement(
            model,
            "model",
            {"name": packet_name},
        )

        ET.SubElement(packet, "pose").text = (
            f"{WATER_X:.3f} "
            f"{WATER_Y:.3f} "
            f"{WATER_Z:.3f} 0 0 0"
        )

        ET.SubElement(packet, "static").text = "false"

        link = ET.SubElement(
            packet,
            "link",
            {"name": "water"},
        )

        ET.SubElement(link, "gravity").text = "true"

        inertial = ET.SubElement(link, "inertial")

        ET.SubElement(
            inertial,
            "mass",
        ).text = f"{PACKET_MASS:.6f}"

        inertia = ET.SubElement(inertial, "inertia")

        ET.SubElement(
            inertia,
            "ixx",
        ).text = f"{packet_inertia:.8f}"

        ET.SubElement(
            inertia,
            "iyy",
        ).text = f"{packet_inertia:.8f}"

        ET.SubElement(
            inertia,
            "izz",
        ).text = f"{packet_inertia:.8f}"

        # No collision and no visual:
        # detached water packets are invisible and cannot strike the drone.
        plugin = ET.SubElement(
            model,
            "plugin",
            {
                "filename": "gz-sim-detachable-joint-system",
                "name": "gz::sim::systems::DetachableJoint",
            },
        )

        ET.SubElement(
            plugin,
            "parent_link",
        ).text = "base_link"

        ET.SubElement(
            plugin,
            "child_model",
        ).text = packet_name

        ET.SubElement(
            plugin,
            "child_link",
        ).text = "water"

        ET.SubElement(
            plugin,
            "detach_topic",
        ).text = f"/skyscrub/water/detach/{i:02d}"

        ET.SubElement(
            plugin,
            "attach_topic",
        ).text = f"/skyscrub/water/attach/{i:02d}"

        ET.SubElement(
            plugin,
            "output_topic",
        ).text = f"/skyscrub/water/state/{i:02d}"

    ET.indent(tree, space="  ")

    tree.write(
        SDF_FILE,
        encoding="UTF-8",
        xml_declaration=True,
    )

    # Static builder just generated EMPTY, therefore it set the
    # airframe hover value for 18.893 kg. Dynamic flight starts FULL,
    # so reset the initial feed-forward value to the FULL mass.
    required_thrust_per_motor = (
        full_mass * GRAVITY / 6.0
    )

    required_omega = math.sqrt(
        required_thrust_per_motor / MOTOR_CONSTANT
    )

    full_hover = (
        required_omega - ESC_MIN
    ) / (
        ESC_MAX - ESC_MIN
    )

    airframe = AIRFRAME_FILE.read_text()

    airframe = replace_parameter(
        airframe,
        "MPC_THR_HOVER",
        f"{full_hover:.3f}",
    )

    AIRFRAME_FILE.write_text(airframe)

    print("=" * 72)
    print("SKYSCRUB V0.2 DYNAMIC WATER MODEL")
    print("=" * 72)

    print(f"Dry mass                    : {dry_mass:.3f} kg")
    print(f"Water packet mass           : {PACKET_MASS:.3f} kg")
    print(f"Number of packets           : {PACKET_COUNT}")
    print(
        f"Total attached water        : "
        f"{PACKET_COUNT * PACKET_MASS:.3f} kg"
    )
    print(f"Initial total mass          : {full_mass:.3f} kg")
    print(f"Initial MPC_THR_HOVER       : {full_hover:.3f}")

    print()
    print("First detach topic:")
    print("  /skyscrub/water/detach/00")

    print()
    print("Last detach topic:")
    print("  /skyscrub/water/detach/31")

    print("=" * 72)


if __name__ == "__main__":
    main()
