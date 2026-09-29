#!/usr/bin/env python3
"""Author the measured plate assembly into an Isaac Lab plate-EE USD.

The plate collider in the current dual-FR3 USD is a child of link7, rather
than an independently articulated rigid body.  PhysX therefore needs the
*combined* stock-link7 + fixed-plate spatial inertia on link7.  This script
does not attach a second MassAPI to the collider, which would be ambiguous
and can double-count the load.

Run with the Python distributed with Isaac Sim/Isaac Lab:

    /home/dyros/anaconda3/envs/env_isaaclab/bin/python \
      scripts/tools/update_isaaclab_plate_ee_inertia.py \
      --usd /home/dyros/dual_fr3_lab/source/dual_fr3_lab/dual_fr3_lab/assets/dual_fr3_plate_ee.usd \
      --write

The default is a dry run.  The physical plate is modelled as a uniform
140 x 140 x 25 mm rectangular solid with the measured all-in mass 0.1324 kg.
Its centre is z=0.107+0.0125 m in the link7 frame.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdPhysics


PLATE_MASS_KG = 0.1324
PLATE_COM_IN_LINK7_M = np.array([0.0, 0.0, 0.107 + 0.0125])
# Uniform 0.140 x 0.140 x 0.025 m rectangular solid, about its own COM.
PLATE_INERTIA_KGM2 = np.diag([0.00022314916666666668,
                               0.00022314916666666668,
                               0.00043250666666666675])

# Factory FR3 link7 properties from franka_description/robots/fr3/inertials.yaml.
LINK7_MASS_KG = 0.6271432862
LINK7_COM_M = np.array([0.0045225817, 0.0086261921, -0.0161633251])
LINK7_INERTIA_KGM2 = np.array([
    [0.00021092389150104718, -0.00002433299114461931, 0.00004564480393778983],
    [-0.00002433299114461931, 0.00017718568002411474, 0.00008744070223226438],
    [0.00004564480393778983, 0.00008744070223226438, 0.00005993190599659971],
])


def _parallel_axis(inertia: np.ndarray, mass: float, displacement: np.ndarray) -> np.ndarray:
    """Move a COM inertia tensor by ``displacement`` to a common COM."""
    return inertia + mass * (
        float(displacement @ displacement) * np.eye(3) - np.outer(displacement, displacement)
    )


def _matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Return a scalar-first unit quaternion whose rotation matrix is matrix."""
    m = matrix
    trace = float(np.trace(m))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m[2, 1] - m[1, 2]) / scale
        y = (m[0, 2] - m[2, 0]) / scale
        z = (m[1, 0] - m[0, 1]) / scale
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / scale
        x = 0.25 * scale
        y = (m[0, 1] + m[1, 0]) / scale
        z = (m[0, 2] + m[2, 0]) / scale
    elif m[1, 1] > m[2, 2]:
        scale = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / scale
        x = (m[0, 1] + m[1, 0]) / scale
        y = 0.25 * scale
        z = (m[1, 2] + m[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / scale
        x = (m[0, 2] + m[2, 0]) / scale
        y = (m[1, 2] + m[2, 1]) / scale
        z = 0.25 * scale
    q = np.array([w, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    # q and -q represent the same frame; use a stable, readable convention.
    if q[0] < 0.0:
        q *= -1.0
    return tuple(float(value) for value in q)


def combined_link7_inertia() -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    """Return mass, COM, principal moments, principal-axes rotation for link7."""
    total_mass = LINK7_MASS_KG + PLATE_MASS_KG
    total_com = (
        LINK7_MASS_KG * LINK7_COM_M + PLATE_MASS_KG * PLATE_COM_IN_LINK7_M
    ) / total_mass
    tensor = _parallel_axis(
        LINK7_INERTIA_KGM2, LINK7_MASS_KG, LINK7_COM_M - total_com
    ) + _parallel_axis(
        PLATE_INERTIA_KGM2, PLATE_MASS_KG, PLATE_COM_IN_LINK7_M - total_com
    )
    moments, axes = np.linalg.eigh(tensor)
    # Eigenvectors are the columns of axes.  A reflection is not a valid
    # orientation quaternion, so flip one axis when needed.
    if np.linalg.det(axes) < 0.0:
        axes[:, 0] *= -1.0
    return total_mass, total_com, moments, axes


def _format(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:.10g}" for value in values) + "]"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", type=Path, required=True, help="dual_fr3_plate_ee.usd to update")
    parser.add_argument("--write", action="store_true", help="write changes; omit for a dry run")
    args = parser.parse_args()

    if not args.usd.is_file():
        raise SystemExit(f"USD does not exist: {args.usd}")

    mass, com, moments, axes = combined_link7_inertia()
    q_wxyz = _matrix_to_quaternion(axes)
    print("Physical plate: mass=0.1324 kg, COM(link7)=[0, 0, 0.1195] m")
    print(f"Combined link7: mass={mass:.10g} kg, COM={_format(com)} m")
    print(f"Principal inertia={_format(moments)} kg m^2")
    print(f"Principal axes quaternion (w,x,y,z)={_format(np.array(q_wxyz))}")

    stage = Usd.Stage.Open(str(args.usd))
    if stage is None:
        raise SystemExit(f"Could not open USD: {args.usd}")

    for side in ("left", "right"):
        path = f"/fr3/{side}_fr3_link7"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise SystemExit(f"Missing expected link7 prim: {path}")
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        before = mass_api.GetMassAttr().Get()
        print(f"{path}: existing mass={before!r} kg")
        if args.write:
            mass_api.GetMassAttr().Set(float(mass))
            mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(*com))
            mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*moments))
            mass_api.GetPrincipalAxesAttr().Set(
                Gf.Quatf(q_wxyz[0], Gf.Vec3f(*q_wxyz[1:]))
            )
            # Keep the modelling assumption inspectable even though mass is
            # authored on the rigid parent rather than on its collider child.
            prim.SetCustomDataByKey("physical_plate_ee_mass_kg", PLATE_MASS_KG)
            # USD custom dictionaries do not accept a generic Python list;
            # retain these non-physical provenance fields as readable text.
            prim.SetCustomDataByKey("physical_plate_ee_com_in_link7_m", _format(PLATE_COM_IN_LINK7_M))
            prim.SetCustomDataByKey(
                "physical_plate_ee_inertia_about_com_kgm2",
                _format(np.diag(PLATE_INERTIA_KGM2)),
            )

    if not args.write:
        print("Dry run only. Re-run with --write to update the USD.")
        return
    stage.Save()
    print(f"Updated and saved: {args.usd}")


if __name__ == "__main__":
    main()
