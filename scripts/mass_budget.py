#!/usr/bin/env python3

import csv
from pathlib import Path

root = Path(__file__).resolve().parents[1]
csv_file = root / "config" / "mass_budget_v0.2.csv"

verified = 0.0
provisional = 0.0
unknown = []

print("=" * 68)
print("SKYSCRUB V0.2 MASS BUDGET")
print("=" * 68)

with csv_file.open() as f:
    reader = csv.DictReader(f)

    for row in reader:
        name = row["component"]
        qty = float(row["quantity"])
        mass_text = row["mass_each_kg"].strip()
        status = row["status"].strip()

        if not mass_text:
            unknown.append(name)
            continue

        mass = float(mass_text) * qty

        if status == "VERIFIED":
            verified += mass
        elif status == "PROVISIONAL":
            provisional += mass

        print(f"{name:43s} {mass:7.3f} kg  [{status}]")

planning_dry_mass = verified + provisional

print()
print("-" * 68)
print(f"Verified subtotal                     : {verified:7.3f} kg")
print(f"Provisional subtotal                  : {provisional:7.3f} kg")
print(f"Current planning dry subtotal         : {planning_dry_mass:7.3f} kg")
print(f"With full 8 L water                   : {planning_dry_mass + 8.0:7.3f} kg")
print(f"With half 4 L water                   : {planning_dry_mass + 4.0:7.3f} kg")
print(f"With empty tank                       : {planning_dry_mass:7.3f} kg")

print()
print("Still unresolved:")
for item in unknown:
    print(f"  - {item}")

print()
print("IMPORTANT:")
print("These totals do NOT yet include the unresolved components above.")
print("Do not use them as the final SkyScrub takeoff mass.")
print("=" * 68)
