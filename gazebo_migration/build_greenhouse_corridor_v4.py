#!/usr/bin/env python3
"""Build the first 8 m of the measured corridor, starting at its low end."""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET

import yaml

from build_greenhouse_corridor_v3 import BASE, CONFIG, MODELS, add, include, new_model, vec, write_model

WORLD_NAME = "tomato_corridor_v4"
SECTION_LENGTH = 8.0
SOIL_FREEBOARD = 0.02
RAIL_TOP_Z = 0.04


def add_box(link, name, center, size, color):
    for kind in ("visual", "collision"):
        part = add(link, kind, attrs={"name": f"{name}_{kind}"})
        add(part, "pose", vec((*center, 0, 0, 0)))
        add(add(add(part, "geometry"), "box"), "size", vec(size))
        if kind == "visual":
            material = add(part, "material")
            add(material, "ambient", color)
            add(material, "diffuse", color)


def build_fixtures(planter, support, track):
    def support_body(model):
        height = support["cross_section_height"]
        link = add(model, "link", attrs={"name": "support"})
        add_box(link, "beam", (0, 0, -height / 2),
                (SECTION_LENGTH, support["cross_section_width"], height), "0.38 0.42 0.40 1")

    def planter_body(model):
        link = add(model, "link", attrs={"name": "planter"})
        length, width, height = (planter[key] for key in ("length", "width", "height"))
        wall = 0.008
        color = "0.55 0.27 0.13 1"
        add_box(link, "base", (0, 0, wall / 2), (length, width, wall), color)
        for side in (-1, 1):
            add_box(link, f"side_{side}", (0, side * (width - wall) / 2, height / 2),
                    (length, wall, height), color)
            add_box(link, f"end_{side}", (side * (length - wall) / 2, 0, height / 2),
                    (wall, width - 2 * wall, height), color)
        soil_height = height - SOIL_FREEBOARD
        add_box(link, "soil", (0, 0, (wall + soil_height) / 2),
                (length - 2 * wall, width - 2 * wall, soil_height - wall), "0.24 0.16 0.10 1")

    def track_body(model):
        link = add(model, "link", attrs={"name": "rails"})
        radius = track["rail_diameter"] / 2
        gauge = track["rail_outer_width"] - 2 * radius
        for side in (-1, 1):
            rail = add(link, "visual", attrs={"name": f"rail_{side}"})
            add(rail, "pose", vec((0, side * gauge / 2, RAIL_TOP_Z - radius, 0, math.pi / 2, 0)))
            cylinder = add(add(rail, "geometry"), "cylinder")
            add(cylinder, "radius", radius)
            add(cylinder, "length", SECTION_LENGTH)
            add(add(rail, "material"), "diffuse", "0.48 0.50 0.53 1")

    for name, populate in (("measured_support_v4", support_body),
                           ("measured_planter_v4", planter_body), ("robot_track_v4", track_body)):
        write_model(name, new_model(name, populate))


def build_plant_variants():
    variants = {}
    for side in range(2):
        for index in range(10):
            source = f"corridor_plant_s{side}_i{index}"
            name = f"corridor_v4_plant_s{side}_i{index}"
            root = ET.parse(MODELS / source / "model.sdf").getroot()
            model = root.find("model")
            model.set("name", name)
            shape_count = 0
            for link in model.findall("link"):
                for collision in link.findall("collision"):
                    # Keep the main stem mesh and the original fruit spheres.
                    # The dense cylinder/cap chains belong to the source's fine contact model.
                    geometry = collision.find("geometry")
                    is_stem_mesh = geometry.find("mesh") is not None
                    is_fruit = (geometry.find("sphere") is not None
                                and collision.get("name", "").startswith("g_tr"))
                    if is_stem_mesh or is_fruit:
                        shape_count += 1
                    else:
                        link.remove(collision)
            ET.indent(root, space="  ")
            write_model(name, root)
            variants[side, index] = (name, shape_count)
    return variants


def main():
    measurements = yaml.safe_load(CONFIG.read_text())["measurements"]
    planter, support, track = (measurements[key] for key in ("planter", "support", "track"))
    spacing = (support["length"] - planter["length"]) / (planter["pots_per_support"] - 1)
    # Distances and spacing are measured along the support. Positive X is uphill.
    low = support["bottom_height_at_x_min"]
    grade = (support["bottom_height_at_x_max"] - low) / support["length"]
    angle = math.asin(grade)
    pitch = -angle  # SDF rotation about +Y maps +X downhill for positive pitch.
    c = math.cos(angle)
    pot_count = math.floor((SECTION_LENGTH - planter["length"]) / spacing) + 1
    offset = track["planter_center_offset_from_track_center"]
    build_fixtures(planter, support, track)
    variants = build_plant_variants()

    root = ET.parse(BASE / "tomato_corridor_v2.world").getroot()
    world = root.find("world")
    world.set("name", WORLD_NAME)
    for element in world.findall("include"):
        if element.findtext("name") != "greenhouse":
            world.remove(element)
    world.find("gui/camera/pose").text = "6 -5 3.6 0 0.31 2.43"
    include(world, "model://robot_track_v4", "robot_track_v4", (0, 0, 0, 0, 0, 0))

    def on_bed(distance, y, height=0.0):
        return ((distance - SECTION_LENGTH / 2) * c - height * grade,
                y, low + distance * grade + height * c)

    introduced_plant_collisions = 0
    for side, y in enumerate((-offset, offset)):
        include(world, "model://measured_support_v4", f"support_{side}",
                (*on_bed(SECTION_LENGTH / 2, y), 0, pitch, 0))
        for index in range(pot_count):
            station = planter["length"] / 2 + index * spacing
            include(world, "model://measured_planter_v4", f"planter_{side}_{index}",
                    (*on_bed(station, y), 0, pitch, 0))
            for pair in range(planter["plants_per_pot"]):
                root_station = station + (pair - 0.5) * planter["plant_root_spacing"]
                # Each row already has its own plant orientation in the v2 instances.
                source, shape_count = variants[side, (2 * index + pair) % 10]
                introduced_plant_collisions += shape_count
                include(world, f"model://{source}", f"plant_{side}_{index}_{pair}",
                        (*on_bed(root_station, y, planter["height"] - SOIL_FREEBOARD), 0, 0, 0))

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(BASE / f"{WORLD_NAME}.world", encoding="utf-8", xml_declaration=True)
    report = {
        "length_m": SECTION_LENGTH, "pot_bottom_end_heights_m": [low, low + SECTION_LENGTH * grade],
        "support_pitch_deg": math.degrees(angle), "row_centers_y_m": [-offset, offset],
        "planter_spacing_m": spacing, "far_end_clearance_m": SECTION_LENGTH - (
            (pot_count - 1) * spacing + planter["length"]),
        "planters": 2 * pot_count, "plants": 2 * pot_count * planter["plants_per_pot"],
        "plant_collision_shapes": introduced_plant_collisions,
        "plant_collision_geometry": "source stem mesh and fruit spheres",
        "rail_outer_width_m": track["rail_outer_width"], "rail_diameter_m": track["rail_diameter"],
        "ceiling_lamps": 0,
    }
    (BASE / f"{WORLD_NAME}_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
