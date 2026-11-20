#!/usr/bin/env python3
"""Generate an isolated Gazebo Classic world from field measurements (v3)."""
from __future__ import annotations

import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh
import yaml

BASE = Path(__file__).resolve().parent
ASSETS = BASE / "assets"
MODELS = ASSETS / "models"
CONFIG = BASE / "layout_config_v3.yaml"


def add(parent, tag, value=None, attrs=None):
    node = ET.SubElement(parent, tag, attrs or {})
    if value is not None:
        node.text = str(value)
    return node


def vec(values):
    return " ".join(f"{float(v):.10g}" for v in values)


def new_model(name, body):
    root = ET.Element("sdf", {"version": "1.7"})
    model = ET.SubElement(root, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    body(model)
    ET.indent(root, space="  ")
    return root


def write_model(name, root):
    directory = MODELS / name
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "model.sdf").write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    config = ET.Element("model", {"name": name})
    add(config, "name", name)
    add(config, "version", "1.0")
    add(config, "sdf", "model.sdf", {"version": "1.7"})
    ET.indent(config, space="  ")
    (directory / "model.config").write_bytes(ET.tostring(config, encoding="utf-8", xml_declaration=True))


def build_support(width, height):
    def body(model):
        link = ET.SubElement(model, "link", {"name": "support"})
        collision = ET.SubElement(link, "collision", {"name": "support_collision"})
        ET.SubElement(collision, "geometry")
        box = ET.SubElement(collision.find("geometry"), "box")
        add(box, "size", vec([32.0, width, height]))
        visual = ET.SubElement(link, "visual", {"name": "support_visual"})
        geometry = ET.SubElement(visual, "geometry")
        add(ET.SubElement(geometry, "box"), "size", vec([32.0, width, height]))
        material = ET.SubElement(visual, "material")
        add(material, "ambient", "0.23 0.25 0.24 1")
        add(material, "diffuse", "0.38 0.42 0.40 1")
    write_model("measured_support_v3", new_model("measured_support_v3", body))


def build_planter(length, width, height):
    source = MODELS / "flowerpot_model/meshes/maceta.dae"
    mesh = trimesh.load(source, force="scene", process=False)
    bounds = mesh.bounds
    source_size = bounds[1] - bounds[0]
    scale = [length / source_size[0], width / source_size[1], height / source_size[2]]
    visual_offset = [-bounds[0][i] * scale[i] for i in range(3)]

    def body(model):
        link = ET.SubElement(model, "link", {"name": "planter"})
        visual = ET.SubElement(link, "visual", {"name": "planter_visual"})
        ET.SubElement(visual, "pose").text = f"{vec(visual_offset)} 0 0 0"
        geometry = ET.SubElement(visual, "geometry")
        mesh_node = ET.SubElement(geometry, "mesh")
        add(mesh_node, "uri", "model://flowerpot_model/meshes/maceta.dae")
        add(mesh_node, "scale", vec(scale))
        material = ET.SubElement(visual, "material")
        add(material, "ambient", "0.40 0.21 0.13 1")
        add(material, "diffuse", "0.67 0.36 0.22 1")
        wall = 0.012
        boxes = [
            (0, 0, 0.006, length, width, 0.012),
            (0, -width / 2 + wall / 2, height / 2, length, wall, height),
            (0, width / 2 - wall / 2, height / 2, length, wall, height),
            (-length / 2 + wall / 2, 0, height / 2, wall, width, height),
            (length / 2 - wall / 2, 0, height / 2, wall, width, height),
        ]
        for index, (x, y, z, sx, sy, sz) in enumerate(boxes):
            collision = ET.SubElement(link, "collision", {"name": f"wall_{index}"})
            ET.SubElement(collision, "pose").text = f"{vec([x, y, z])} 0 0 0"
            geom = ET.SubElement(collision, "geometry")
            add(ET.SubElement(geom, "box"), "size", vec([sx, sy, sz]))
    write_model("measured_planter_v3", new_model("measured_planter_v3", body))
    return {"visual_scale": scale, "source_mesh_size_m": source_size.tolist()}


def build_track(diameter, outer_width, segment_length=0.906):
    center_spacing = outer_width - diameter
    count = math.ceil(32.0 / segment_length)
    total = 32.0
    sources = [BASE.parent / "meshes/env/pipe_a.stl", BASE.parent / "meshes/env/pipe_b.stl"]
    directory = MODELS / "robot_track_v3"
    mesh_dir = directory / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for source, name in zip(sources, ["pipe_a.stl", "pipe_b.stl"]):
        shutil.copy2(source, mesh_dir / name)
    root = ET.Element("sdf", {"version": "1.7"})
    model = ET.SubElement(root, "model", {"name": "robot_track_v3"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "rails"})
    for segment in range(count):
        x = -total / 2 + (segment + 0.5) * total / count
        for side, y in enumerate([-center_spacing / 2, center_spacing / 2]):
            name = f"rail_{segment}_{side}"
            visual = ET.SubElement(link, "visual", {"name": name})
            z_max = (20.05272102 if side == 0 else 23.82342911) * 0.00125
            ET.SubElement(visual, "pose").text = f"{x:.10g} {y:.10g} {0.04 - z_max:.10g} 0 0 0"
            geometry = ET.SubElement(visual, "geometry")
            mesh = ET.SubElement(geometry, "mesh")
            add(mesh, "uri", f"model://robot_track_v3/meshes/pipe_{'a' if side == 0 else 'b'}.stl")
            add(mesh, "scale", vec([total / count / segment_length, 0.00125, 0.00125]))
            material = ET.SubElement(visual, "material")
            add(material, "ambient", "0.24 0.25 0.27 1")
            add(material, "diffuse", "0.48 0.50 0.53 1")
    ET.indent(root, space="  ")
    (directory / "model.sdf").write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    config = ET.Element("model", {"name": "robot_track_v3"})
    add(config, "name", "robot_track_v3")
    add(config, "version", "1.0")
    add(config, "sdf", "model.sdf", {"version": "1.7"})
    ET.indent(config, space="  ")
    (directory / "model.config").write_bytes(ET.tostring(config, encoding="utf-8", xml_declaration=True))
    return center_spacing


def build_greenhouse(length):
    source = MODELS / "greenhouse_structure"
    target = MODELS / "greenhouse_v3"
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target)
    sdf_path = target / "model.sdf"
    root = ET.parse(sdf_path).getroot()
    mesh_path = target / "meshes/Structure.dae"
    mesh = trimesh.load(mesh_path, force="scene", process=False)
    current_length = float(mesh.bounds[1][0] - mesh.bounds[0][0])
    ratio = length / current_length
    for scale_node in root.findall(".//geometry/mesh/scale"):
        scale = [float(value) for value in scale_node.text.split()]
        scale[0] *= ratio
        scale_node.text = vec(scale)
    for uri in root.findall(".//geometry/mesh/uri"):
        uri.text = uri.text.replace("model://greenhouse_structure/", "model://greenhouse_v3/")
    ET.indent(root, space="  ")
    sdf_path.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    config = ET.Element("model", {"name": "greenhouse_v3"})
    add(config, "name", "greenhouse_v3")
    add(config, "version", "1.0")
    add(config, "sdf", "model.sdf", {"version": "1.6"})
    ET.indent(config, space="  ")
    (target / "model.config").write_bytes(ET.tostring(config, encoding="utf-8", xml_declaration=True))
    return {"original_visual_length_m": current_length, "longitudinal_scale": ratio}


def include(world, uri, name, pose):
    node = ET.SubElement(world, "include")
    add(node, "uri", uri)
    add(node, "name", name)
    add(node, "pose", vec(pose))


def build_world(cfg):
    m = cfg["measurements"]
    planter, support, track = m["planter"], m["support"], m["track"]
    plant_rows = m["plant_rows"]
    length = support["length"]
    count = planter["pots_per_support"]
    pot_length = planter["length"]
    pitch = math.atan2(support["bottom_height_at_x_max"] - support["bottom_height_at_x_min"], length)
    center_spacing = (length - pot_length) / (count - 1)
    x_start = -length / 2 + pot_length / 2
    row_y = [-track["planter_center_offset_from_track_center"], track["planter_center_offset_from_track_center"]]
    root = ET.Element("sdf", {"version": "1.7"})
    world = ET.SubElement(root, "world", {"name": "tomato_corridor_v3"})
    physics = ET.SubElement(world, "physics", {"name": "default_physics", "type": "ode"})
    add(physics, "max_step_size", "0.001")
    add(physics, "real_time_factor", "1")
    scene = ET.SubElement(world, "scene")
    add(scene, "ambient", "0.72 0.72 0.72 1")
    add(scene, "background", "0.80 0.84 0.88 1")
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    glink = ET.SubElement(ground, "link", {"name": "ground"})
    collision = ET.SubElement(glink, "collision", {"name": "ground_collision"})
    plane = ET.SubElement(ET.SubElement(collision, "geometry"), "plane")
    add(plane, "normal", "0 0 1")
    add(plane, "size", "80 20")
    visual = ET.SubElement(glink, "visual", {"name": "ground_visual"})
    plane = ET.SubElement(ET.SubElement(visual, "geometry"), "plane")
    add(plane, "normal", "0 0 1")
    add(plane, "size", "80 20")
    sun = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    add(sun, "cast_shadows", "true")
    add(sun, "pose", "0 0 10 0 0 0")
    add(sun, "direction", "-0.3 0.2 -1")
    gui = ET.SubElement(world, "gui")
    camera = ET.SubElement(gui, "camera", {"name": "user_camera"})
    add(camera, "pose", "23 -20 13 0 -0.34 2.425")
    include(world, "model://robot_track_v3", "robot_track_v3", [0, 0, 0, 0, 0, 0])
    include(world, "model://greenhouse_v3", "greenhouse", [0, 0, 0, 0, 0, 0])
    support_z_mid = (support["bottom_height_at_x_min"] + support["bottom_height_at_x_max"]) / 2
    for side, y in enumerate(row_y):
        include(world, "model://measured_support_v3", f"support_{side}",
                [0, y, support_z_mid - support["cross_section_height"] / 2, 0, pitch, 0])
        for index in range(count):
            x = x_start + index * center_spacing
            bottom_z = support["bottom_height_at_x_min"] + (x + length / 2) / length * (
                support["bottom_height_at_x_max"] - support["bottom_height_at_x_min"])
            stem_yaw = 0 if side == 0 else math.pi
            include(world, "model://measured_planter_v3", f"planter_{side}_{index}",
                    [x, y, bottom_z, 0, pitch, 0])
            for pair in range(planter["plants_per_pot"]):
                plant_x = x + (-0.5 if pair == 0 else 0.5) * planter["plant_root_spacing"]
                variant = (index * 2 + pair + side * 7) % 20
                source_model = f"corridor_plant_s{variant // 10}_i{variant % 10}"
                root_z = bottom_z + pot_planter_height(planter)
                include(world, f"model://{source_model}", f"plant_{side}_{index}_{pair}",
                        [plant_x, y, root_z, 0, pitch, stem_yaw])
    ET.indent(root, space="  ")
    path = BASE / "tomato_corridor_v3.world"
    path.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    return {"world": str(path), "pots": count * len(row_y), "plants": count * len(row_y) * planter["plants_per_pot"],
            "row_y_m": row_y, "planter_center_spacing_m": center_spacing, "pitch_rad": pitch,
            "pitch_deg": math.degrees(pitch), "bottom_z_at_x_ends_m": [support["bottom_height_at_x_min"],
                                                                          support["bottom_height_at_x_max"]],
            "lamp_models": 0}


def pot_planter_height(planter):
    return planter["height"]


def main():
    cfg = yaml.safe_load(CONFIG.read_text())
    m = cfg["measurements"]
    build_support(m["support"]["cross_section_width"], m["support"]["cross_section_height"])
    planter_report = build_planter(m["planter"]["length"], m["planter"]["width"], m["planter"]["height"])
    track_spacing = build_track(m["track"]["rail_diameter"], m["track"]["rail_outer_width"])
    greenhouse_report = build_greenhouse(m["support"]["length"])
    report = build_world(cfg)
    report["track_center_spacing_m"] = track_spacing
    report["planter_visual"] = planter_report
    report["greenhouse"] = greenhouse_report
    report["expected_track_outer_width_m"] = track_spacing + m["track"]["rail_diameter"]
    output = BASE / "tomato_corridor_v3_report.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
