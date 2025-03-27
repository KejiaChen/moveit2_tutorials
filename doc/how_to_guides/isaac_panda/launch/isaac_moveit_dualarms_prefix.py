import sys
import re
import os

import carb
import numpy as np
from pathlib import Path

from isaacsim import SimulationApp

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

# import omni.client

# from isaacsim.storage.native import nucleus

from isaacsim.core.utils.prims import ( create_prim, 
                                        move_prim,
                                        get_prim_attribute_names,
                                        get_prim_attribute_value,
                                        set_prim_attribute_value )
from pxr import Usd, Sdf, UsdPhysics, Gf, UsdUtils
import omni.usd


# Paths
# FRANKA_STAGE_PATH = "/Franka"
RENAME_PREFIX = "right"
FRANKA_USD_PATH = "/home/tp2/ws_humble/src/moveit2_tutorials/doc/how_to_guides/isaac_panda/assets/"
FRANKA_USD_NAME = "franka_flattened.usd"
FRANKA_USD_RENAMED_NAME = RENAME_PREFIX + "_" + FRANKA_USD_NAME

def rename_panda_subprims(franka_path, prefix):
    """
    Traverse all descendants under `franka_path`, renaming any prim that starts
    with "panda_" (or equals "rootjoint") by prepending `prefix + "_"`.

    Also fixes references in physics joint attributes.
    """

    root_prim = stage.GetPrimAtPath(franka_path)
    if not root_prim.IsValid():
        print(f"[ERROR] No valid prim at {franka_path}")
        return

    # 1) Gather rename operations in a list
    rename_ops = []
    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim:
            continue
        old_name = prim.GetName()
        if old_name.startswith("panda_"):
            old_path = prim.GetPath()
            # print(f"Found {old_name} at {old_path}")
            new_name = f"{prefix}_{old_name}"
            new_path = old_path.GetParentPath().AppendChild(new_name)
            rename_ops.append((old_path, new_path))
    
    # 2) Sort from deepest to shallowest
    rename_ops.sort(key=lambda item: item[0].pathElementCount, reverse=True)

    # 3) Apply renaming
    for (old_path, new_path) in rename_ops:
        if stage.GetPrimAtPath(old_path):
            move_prim(str(old_path), str(new_path))

    # dict
    rename_map = {str(old): str(new) for (old, new) in rename_ops}
    # print("remap keys:", rename_map.keys())

    # 4) Fix references in joint attributes
    #    Depending on your version of Isaac Sim, joint attributes might be
    #    "physics:body0"/"physics:body1" or "joint:parent"/"joint:child".
    for prim in Usd.PrimRange(root_prim):
        prim_path = str(prim.GetPath())
        # print(f"Checking {prim_path}...")
        
        if "joint" in prim.GetName().lower():
            print(f"Checking {prim_path}...")
            for rel in prim.GetRelationships():
                old_targets = rel.GetTargets()
                new_targets = []
                changed = False
                for t in old_targets:
                    t_str = str(t)
                    print(f"  -> {t_str}")
                    # If we renamed this link
                    if t_str in rename_map.keys():
                        new_targets.append(Sdf.Path(rename_map[t_str]))
                        changed = True
                    else:
                        new_targets.append(t)
                if changed:
                    rel.ClearTargets(True)
                    rel.SetTargets(new_targets)
                    print(f"Updated {rel.GetName()} in {prim_path}")

omni.usd.get_context().open_stage(FRANKA_USD_PATH+FRANKA_USD_NAME)

stage = omni.usd.get_context().get_stage()

# print all prims
# for prim in stage.Traverse():
#     print(prim.GetPath())

# prim = stage.GetPrimAtPath("/panda/panda_link6/panda_joint7")

# for rel in prim.GetRelationships():
#     print("Relationship:", rel.GetName())
#     for target in rel.GetTargets():
#         print("  ->", str(target))

# # Reference Franka for the left arm
# left_arm_prim = create_prim(
#     prim_path=FRANKA_STAGE_PATH,         # top-level path
#     usd_path=FRANKA_USD_PATH,         # or an Omniverse asset path
#     position=Gf.Vec3d(0.0, -0.28, 1.0),
# )

rename_panda_subprims("/panda", RENAME_PREFIX)

# delete root joint, to be added later to base
# stage.RemovePrim("/panda/rootJoint")

# save the stage
stage.GetRootLayer().Export(FRANKA_USD_PATH+FRANKA_USD_RENAMED_NAME)
print(f"Saved renamed usd to {FRANKA_USD_PATH+FRANKA_USD_RENAMED_NAME}")