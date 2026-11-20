#!/usr/bin/env python3
"""Rebuild the fruit-bearing green meshes while reusing the measured v4 layout."""
from copy import deepcopy
import json
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline
import trimesh

from build_greenhouse_corridor_v3 import BASE, MODELS, write_model
from natural_peduncle import HermiteTube, obj_mesh, unit

WORLD_V4 = BASE / "tomato_corridor_v4.world"
WORLD_V5 = BASE / "tomato_corridor_v5.world"
COMMON = BASE / "assets/tomato_v2_common_assets"
PLANT_V4 = re.compile(r"model://(corridor_v4_plant_s\d+_i\d+)")
GREEN = {"ambient": "0.32 0.42 0.22 1", "diffuse": "0.44 0.56 0.30 1",
         "specular": "0.045 0.045 0.032 1"}


def tube(centers, tangents, radii, sides):
    """Use parallel-transported rings with outward-facing triangles and capped ends."""
    vertices = []
    theta = np.arange(sides) * (2 * np.pi / sides)
    tangents = np.array([unit(vector) for vector in tangents])
    reference = np.eye(3)[np.argmin(np.abs(tangents[0]))]
    normal = unit(np.cross(tangents[0], reference))
    last_tangent = tangents[0]
    for center, tangent, radius in zip(centers, tangents, radii):
        axis = np.cross(last_tangent, tangent)
        normal += np.cross(axis, normal) + np.cross(axis, np.cross(axis, normal)) / (1 + tangent @ last_tangent)
        normal = unit(normal - tangent * (normal @ tangent))
        binormal = np.cross(tangent, normal)
        vertices.extend(center + radius * (np.cos(theta)[:, None] * normal + np.sin(theta)[:, None] * binormal))
        last_tangent = tangent
    faces = []
    for ring in range(len(centers) - 1):
        for j in range(sides):
            a = ring * sides + j
            b = ring * sides + (j + 1) % sides
            faces.extend(((a, b, a + sides), (b, b + sides, a + sides)))
    end = (len(centers) - 1) * sides
    for j in range(1, sides - 1):
        faces.extend(((0, j + 1, j), (end, end + j, end + j + 1)))
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


def peduncle(record):
    curve = HermiteTube.from_record(record)
    fraction = np.linspace(0, 1, 23)
    parameter = curve.parameter(fraction)
    centers = curve.position(parameter) - curve.b
    radii = curve.r_b + (curve.r_a - curve.r_b) * (
        0.73 * (1 - fraction)**2 + 0.27 * (1 - fraction)**15
    )
    return (tube(centers, curve.derivative(parameter), radii, 12), unit(curve.mb),
            curve.position(curve.parameter(.5)), 2 * radii[len(fraction) // 2])


def rachis_curve(shaft, direction):
    """Recover shaft centers from v4 sections. The first axis point is the v4 link origin."""
    bottom = shaft.bounds[0, 2]
    zs = np.r_[np.arange(-.008, bottom + .004, -.010), bottom + .001]
    centers = [np.zeros(3)]
    for z in zs:
        section = shaft.section(plane_origin=(0., 0., z), plane_normal=(0., 0., 1.))
        if section is None or not len(section.vertices):
            continue
        points = section.vertices
        centers.append((points.min(axis=0) + points.max(axis=0)) / 2)
    if len(centers) < 10:
        raise ValueError("Could not recover v4 rachis centerline")
    centers = np.array(centers)
    # Fit the hanging axis with a continuous tangent and the original direction at B.
    spline = CubicSpline(-centers[:, 2], centers, axis=0,
                         bc_type=((1, direction / -direction[2]), "natural"))
    return spline, -centers[-1, 2]


def smooth_pedicel(part, shaft_points, shaft_radii):
    """Identify each proximal mesh ring, then preserve the distal calyx triangles."""
    boundary = part.edges_unique[np.bincount(part.edges_unique_inverse) == 1]
    if len(boundary) < 5:
        return None
    candidates = np.unique(boundary)
    points = part.vertices[candidates]
    distances = np.linalg.norm(points[:, None, :] - shaft_points, axis=2).min(axis=1)
    # Some source components have additional boundary holes on the sepals. Those
    # holes are farther from the hanging axis than the five vertices at the root.
    first = candidates[np.argsort(distances)[:5]]
    if np.ptp(part.vertices[first], axis=0).max() > .003:
        return None
    if not all(sum(n in first for n in part.vertex_neighbors[i]) >= 2 for i in first):
        return None

    root_ring = part.vertices[first]
    root_center = root_ring.mean(axis=0)
    old_radius = np.linalg.norm(root_ring - root_center, axis=1).mean()
    rings = [first]
    centers = [root_center]
    used = set()
    active = set(first)
    for _ in range(8):
        used.update(active)
        active = {n for i in active for n in part.vertex_neighbors[i]} - used
        if len(active) != 5:
            break
        rings.append(np.array(sorted(active)))
        centers.append(part.vertices[rings[-1]].mean(axis=0))
    steps = np.diff(centers, axis=0)
    # The calyx flares back around the fruit. Its first ring is where
    # the original tube turns back sharply, even on stalks with swollen nodes.
    turn = next((i for i in range(2, len(steps))
                 if unit(steps[i - 1]) @ unit(steps[i]) < -.7), None)
    if turn is None:
        return None
    terminal_ring = part.vertices[rings[turn]]
    end = centers[turn]
    radius = np.linalg.norm(terminal_ring - end, axis=1).mean()
    # Place the root inside the remodeled axis without moving the calyx.
    root_index = np.linalg.norm(shaft_points - root_center, axis=1).argmin()
    root = shaft_points[root_index]
    chord = end - root
    length = np.linalg.norm(chord)
    if not .005 <= length <= .065 or np.linalg.norm(root - root_center) > shaft_radii[root_index] + .004:
        return None
    first_tangent = unit(centers[1] - root)
    end_tangent = unit(steps[turn - 1])
    if first_tangent @ chord <= 0 or end_tangent @ chord <= 0:
        return None
    curve = CubicHermiteSpline([0., 1.], [root, end],
                               [first_tangent * length * .92, end_tangent * length * .95])
    fraction = np.linspace(0, 1, max(6, int(np.ceil(length / .0035)) + 1))
    smooth_centers = curve(fraction)
    smooth_tangents = curve(fraction, 1)
    # End inside the preserved calyx rim, without drawing a separate seam.
    smooth_centers = np.vstack((smooth_centers, end + .00055 * end_tangent))
    smooth_tangents = np.vstack((smooth_tangents, end_tangent))
    pedicel_radii = np.interp(fraction, [0., .24, .72, 1.],
                             [old_radius * 1.05, old_radius * .83, old_radius * .70, radius * .90])
    pedicel = tube(smooth_centers, smooth_tangents, np.r_[pedicel_radii, radius * .88], 8)
    calyx = part.copy()
    proximal_vertices = np.concatenate(rings[:turn])
    calyx.update_faces(~np.isin(calyx.faces, proximal_vertices).any(axis=1))
    calyx.remove_unreferenced_vertices()
    return pedicel, calyx


def remodel_rachis(path, direction, base_radius):
    source = obj_mesh(str(path))
    parts = source.split(only_watertight=False)
    shaft = max(parts, key=lambda part: part.extents[2])
    curve, length = rachis_curve(shaft, direction)
    depth = np.linspace(0, length, 44)
    radius = np.interp(depth / length, [0., .13, .48, .80, 1.],
                       [base_radius, .00155, .00128, .00091, .00052])
    points = curve(depth)
    # Keep the mesh overlap at the proximal junction and retain the small tip taper.
    geometry = [tube(np.vstack((-direction * .00045, points)),
                     np.vstack((direction, curve(depth, 1))), np.r_[base_radius, radius], 10)]
    axis_depth = np.linspace(0, length, 601)
    axis_points = curve(axis_depth)
    axis_radii = np.interp(axis_depth, depth, radius)
    updated = 0
    original = 0
    for part in parts:
        if part is shaft:
            continue
        is_branch = len(part.faces) >= 75
        original += is_branch
        pair = smooth_pedicel(part, axis_points, axis_radii) if is_branch else None
        if pair is None:
            geometry.append(part)
        else:
            geometry.extend(pair)
            updated += 1
    return trimesh.util.concatenate(geometry), updated, original


def replace_visual(link, label, mesh, filename, out_dir, model_name):
    originals = [v for v in link.findall("visual") if label in v.get("name", "")]
    visual = ET.Element("visual", {"name": originals[0].get("name")})
    position = list(link).index(originals[0])
    for original in originals:
        link.remove(original)
    link.insert(position, visual)
    ET.SubElement(visual, "pose").text = "0 0 0 0 0 0"
    geometry = ET.SubElement(visual, "geometry")
    uri = ET.SubElement(ET.SubElement(geometry, "mesh"), "uri")
    _ = mesh.vertex_normals
    asset = trimesh.exchange.obj.export_obj(mesh, include_normals=True, include_color=False,
                                            include_texture=False, digits=8)
    (out_dir / filename).write_text(asset + "\n")
    uri.text = f"model://{model_name}/meshes/{filename}"
    material = ET.SubElement(visual, "material")
    for name, value in GREEN.items():
        ET.SubElement(material, name).text = value
    ET.SubElement(material, "double_sided").text = "true"


def check_visual_only(before, after):
    """Assert that everything except the two remodeled visuals has been copied exactly."""
    cleaned = []
    for source in (before, after):
        model = deepcopy(source)
        model.set("name", "plant")
        for link in model.findall("link"):
            for visual in link.findall("visual"):
                if "_rachis_" in visual.get("name") or "_stalk_" in visual.get("name"):
                    link.remove(visual)
        ET.indent(model)
        cleaned.append(ET.tostring(model))
    assert cleaned[0] == cleaned[1], after.get("name")


def build():
    records = {row["instance"]: row for row in
               json.loads((BASE / "randomization_report.json").read_text())["instances"]}
    world = ET.parse(WORLD_V4)
    scene = world.getroot().find("world")
    scene.set("name", WORLD_V5.stem)
    renamed = {}
    placements = []
    for include in scene.findall("include"):
        uri = include.find("uri")
        match = PLANT_V4.fullmatch(uri.text)
        if match is None:
            continue
        source = match[1]
        variant = source.replace("_v4_", "_v5_")
        renamed[source] = variant
        uri.text = f"model://{variant}"
        placements.append((include, source))

    report = {"world": WORLD_V5.name, "plant_instances": len(placements), "variants": [],
              "scope": "Visual meshes only. Existing v4 fruit transforms and collision geometry are preserved."}
    for source, variant in renamed.items():
        row = records[source.replace("_v4_", "_")]
        before = ET.parse(MODELS / source / "model.sdf").getroot()
        tree = deepcopy(before)
        model = tree.find("model")
        model.set("name", variant)
        mesh_dir = MODELS / variant / "meshes"
        mesh_dir.mkdir(parents=True, exist_ok=True)
        entries = []
        for index, truss in enumerate(row["trusses"]):
            link = model.find(f"link[@name='truss_{row['plant']}_{index}']")
            record = truss["peduncle_deformation"]
            link_xyz = np.array([float(x) for x in link.findtext("pose").split()[:3]])
            np.testing.assert_allclose(link_xyz, record["B"], rtol=0, atol=1e-10)
            stalk, direction, cut_center, new_diameter = peduncle(record)
            recorded_local = np.array(truss["cut_center_world_xyz_m"]) - row["root_world_xyz_m"][:3]
            np.testing.assert_allclose(cut_center, recorded_local, rtol=0, atol=1e-10)
            rachis_uri = next(v for v in link.findall("visual") if "_rachis_" in v.get("name"))
            original = rachis_uri.findtext("geometry/mesh/uri").split(f"{COMMON.name}/", 1)[1]
            rachis, updated, branches = remodel_rachis(COMMON / original, direction, record["radius_B_m"])
            if updated < 8:
                raise ValueError(f"{variant}/{index}: only {updated} identifiable pedicels")
            replace_visual(link, "_stalk_", stalk, f"stalk_{index}.obj", mesh_dir, variant)
            replace_visual(link, "_rachis_", rachis, f"rachis_{index}.obj", mesh_dir, variant)
            entries.append({"index": index, "smoothed_pedicels": updated,
                            "preserved_source_pedicel_components": branches - updated,
                            "v4_mid_diameter_mm": 1000 * (record["radius_A_m"] + record["radius_B_m"]),
                            "v5_mid_diameter_mm": 1000 * new_diameter})
        check_visual_only(before.find("model"), model)
        ET.indent(tree, space="  ")
        write_model(variant, tree)
        report["variants"].append({"name": variant, "trusses": entries})
    report["truss_world_truth"] = []
    for include, source in placements:
        row = records[source.replace("_v4_", "_")]
        pose = np.fromstring(include.findtext("pose"), sep=" ")
        np.testing.assert_allclose(pose[3:], 0., rtol=0, atol=1e-10)
        root_xyz = pose[:3]
        translation = root_xyz - row["root_world_xyz_m"][:3]
        entries = []
        for index, truss in enumerate(row["trusses"]):
            deformation = truss["peduncle_deformation"]
            entries.append({
                "index": index,
                "A": (root_xyz + deformation["A"]).tolist(),
                "B": (root_xyz + deformation["B"]).tolist(),
                "cut_center_world_xyz_m": (translation + truss["cut_center_world_xyz_m"]).tolist(),
                "peduncle_axis_world_unit": truss["peduncle_axis_world_unit"],
            })
        report["truss_world_truth"].append({"plant": include.findtext("name"), "trusses": entries})
    report["truss_instances"] = sum(len(plant["trusses"]) for plant in report["truss_world_truth"])
    world.write(WORLD_V5, encoding="utf-8", xml_declaration=True)
    (BASE / "corridor_v5_report.json").write_text(json.dumps(report, indent=2) + "\n")
    mid_v4 = np.median([truss["v4_mid_diameter_mm"] for v in report["variants"] for truss in v["trusses"]])
    mid_v5 = np.median([truss["v5_mid_diameter_mm"] for v in report["variants"] for truss in v["trusses"]])
    print(f"{WORLD_V5.name}: {len(placements)} plants, {report['truss_instances']} trusses, "
          f"{len(renamed)} variants. Peduncle midpoint median diameter {mid_v4:.2f} -> {mid_v5:.2f} mm.")


if __name__ == "__main__":
    build()
