#!/usr/bin/env python3
"""Launch the migration offline, or measure first step and capture OGRE cameras."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parent
MEASURED_CORRIDORS = ("tomato_corridor_v4", "tomato_corridor_v5")


def gazebo_environment(port):
    # Gazebo's Insert panel treats every subdirectory of a model path as
    # a model. Expose the shared resources without exposing assets/models twice.
    shared_models = BASE / "runtime" / "local_models"
    shared_models.mkdir(parents=True, exist_ok=True)
    common = BASE / "assets" / "tomato_v2_common_assets"
    alias = shared_models / common.name
    try:
        alias.symlink_to(common, target_is_directory=True)
    except FileExistsError:
        if alias.resolve() != common:
            raise
    env = os.environ.copy()
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    env["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    env["GAZEBO_IP"] = "127.0.0.1"
    env["GAZEBO_MODEL_PATH"] = f"{BASE}/assets/models:{shared_models}"
    env["GAZEBO_RESOURCE_PATH"] = f"{BASE}/assets:/usr/share/gazebo-11"
    env["GAZEBO_PLUGIN_PATH"] = f"{BASE}/runtime:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
    return env


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def launch_corridor_viewer(world, port, timeout):
    env = gazebo_environment(port)
    with socket.socket() as probe:
        probe.settimeout(0.5)
        if probe.connect_ex(("127.0.0.1", port)) == 0:
            raise SystemExit(f"Gazebo port {port} is already in use. Choose another --port.")

    server = subprocess.Popen(["gzserver", "--verbose", str(world)], env=env)
    client = None
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if server.poll() is not None:
                raise SystemExit(f"Gazebo server exited during world loading ({server.returncode}).")
            try:
                ready = subprocess.run(
                    ["gz", "model", "-w", world.stem, "-m", "ground_plane", "-p"],
                    env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=1,
                )
                if ready.returncode == 0 and len(ready.stdout.split()) == 6:
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(0.25)
        else:
            raise SystemExit(f"Gazebo did not finish loading {world.name} within {timeout:g} s.")

        client = subprocess.Popen(["gzclient", "--verbose"], env=env)
        while client.poll() is None and server.poll() is None:
            time.sleep(0.25)
        return client.returncode if client.returncode is not None else server.returncode
    except KeyboardInterrupt:
        return 130
    finally:
        stop_process(client)
        stop_process(server)


def add_camera(world, name, eye, target, output, fov=0.85):
    direction = [b - a for a, b in zip(eye, target)]
    yaw = math.atan2(direction[1], direction[0])
    pitch = -math.atan2(direction[2], math.hypot(*direction[:2]))
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(map(str, [*eye, 0, pitch, yaw]))
    link = ET.SubElement(model, "link", {"name": "camera_link"})
    sensor = ET.SubElement(link, "sensor", {"name": name, "type": "camera"})
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "2"
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = str(fov)
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = "1920"
    ET.SubElement(image, "height").text = "1440"
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.02"
    ET.SubElement(clip, "far").text = "80"
    save = ET.SubElement(camera, "save", {"enabled": "true"})
    ET.SubElement(save, "path").text = str(output / name)


def measured_truss_camera(world):
    """Match the same end plant, truss and camera pose in both map versions."""
    plant = world.find("include[name='plant_0_0_0']")
    source = plant.findtext("uri").split("/")[-1].replace("_v4_", "_").replace("_v5_", "_")
    records = json.loads((BASE / "randomization_report.json").read_text())["instances"]
    instance = next(row for row in records if row["instance"] == source)
    record = instance["trusses"][1]["peduncle_deformation"]
    root = [float(value) for value in plant.findtext("pose").split()[:3]]
    a, b = record["A"], record["B"]
    target = [(a[i] + b[i]) / 2 + root[i] for i in range(2)] + [b[2] + root[2] - .12]
    eye = [target[0] - .50, target[1] + .17, target[2] + .065]
    return "truss_detail", eye, target, .80


def capture(world_path, port, timeout, truss_detail=False):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    output = BASE / "runtime" / f"{world_path.stem}_{stamp}"
    output.mkdir(parents=True)
    tree = ET.parse(world_path)
    world = tree.getroot().find("world")
    probe = ET.SubElement(world, "plugin", {"name": "runtime_probe", "filename": "libmigration_runtime_probe.so"})
    ET.SubElement(probe, "output").text = str(output / "first_step.json")
    if world_path.stem == "single_plant_verify":
        cameras = [("single_plant", [3.6, -4.8, 2.2], [0.50, -1, 1.62], 1.08),
                   ("reference_9_1", [1.35, -1.95, 1.47], [0.60, -0.96, 1.34], 0.72)]
    elif world_path.stem in MEASURED_CORRIDORS:
        cameras = [("corridor", [5.8, -3.8, 3.5], [0, 0, 1.25], 1.3),
                   ("low_end", [-4.4, 0.0, 1.7], [-2.7, -0.45, 0.95], 1.05)]
        if truss_detail:
            cameras.append(measured_truss_camera(world))
    else:
        cameras = [("corridor", [8.2, -5.5, 3.15], [0.2, 0, 1.6], 1.18)]
        random_report = json.loads((BASE / "randomization_report.json").read_text())
        instances = random_report.get("instances", [])
        choices = [(1,5,1),(0,2,0),(0,3,0),(1,2,0)]
        for camera_index,(side,slot,k) in enumerate(choices):
            instance=next(r for r in instances if r["side"]==side and r["slot"]==slot)
            record=instance["trusses"][k]
            a,b=record["A_world_xyz_m"],record["B_world_xyz_m"]
            target=[(a[0]+b[0])/2,(a[1]+b[1])/2,b[2]-.09]
            delta=record["A_to_B_world_xyz_m"]
            horizontal=math.hypot(delta[0],delta[1]);dx,dy=delta[0]/horizontal,delta[1]/horizontal
            sideways=[.37,-.65,.75,-.45][camera_index]
            outward=[.72,.45,.25,.60][camera_index]
            eye=[target[0]+outward*dx-sideways*dy,target[1]+outward*dy+sideways*dx,target[2]+.07]
            name="truss_detail" if camera_index==0 else f"truss_detail_{camera_index+1}"
            cameras.append((name,eye,target,.88))
    for name, eye, target, fov in cameras:
        add_camera(world, name, eye, target, output, fov)
    derived = output / world_path.name
    tree.write(derived, encoding="utf-8", xml_declaration=True)
    result = {"world": world_path.name, "output": str(output), "timeout_s": timeout}
    env = gazebo_environment(port)
    start = time.monotonic()
    with (output / "gzserver.log").open("w") as log:
        process = subprocess.Popen(["gzserver", "--verbose", str(derived)], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            captured = set()
            while time.monotonic() - start < timeout:
                elapsed = time.monotonic() - start
                if world_path.stem in MEASURED_CORRIDORS:
                    try:
                        high_water = next(line for line in Path(f"/proc/{process.pid}/status").read_text().splitlines()
                                          if line.startswith("VmHWM:"))
                        result["gzserver_peak_rss_kib"] = int(high_water.split()[1])
                    except (OSError, StopIteration):
                        pass
                ready = output / "first_step.json"
                if ready.exists() and "first_step_s" not in result:
                    try:
                        result["runtime_topology"] = json.loads(ready.read_text())
                        result["first_step_s"] = round(elapsed, 3)
                    except json.JSONDecodeError:
                        pass
                for name, *_ in cameras:
                    frames = sorted((output / name).glob("*.jpg")) + sorted((output / name).glob("*.png"))
                    if len(frames) >= 2 and name not in captured:
                        # The second completed frame avoids transient startup scene state.
                        frame = frames[-2]
                        target = output / f"{name}{frame.suffix}"
                        shutil.copy2(frame, target)
                        captured.add(name)
                        result.setdefault("images", {})[name] = str(target)
                        result.setdefault("render_ready_s", {})[name] = round(elapsed, 3)
                if len(captured) == len(cameras) and "first_step_s" in result:
                    result["status"] = "passed"
                    break
                if process.poll() is not None:
                    result["exit_code"] = process.returncode
                    break
                time.sleep(0.1)
            else:
                result["status"] = "timeout"
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("world", choices=MEASURED_CORRIDORS)
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--truss-detail", action="store_true",
                        help="also render a matching v4/v5 close-up in capture mode")
    parser.add_argument("--port", type=int, default=11345)
    parser.add_argument("--timeout", type=float, default=90)
    args = parser.parse_args()
    world = BASE / f"{args.world}.world"
    if args.capture:
        result = capture(world, args.port, args.timeout, args.truss_detail)
        print(json.dumps(result, indent=2), flush=True)
        if result.get("status") != "passed":
            raise SystemExit(1)
    else:
        raise SystemExit(launch_corridor_viewer(world, args.port, args.timeout))


if __name__ == "__main__":
    main()
