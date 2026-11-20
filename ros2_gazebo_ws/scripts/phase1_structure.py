#!/usr/bin/env python3
"""Verify the installed URIs, rail geometry, control plugin and Xacro arguments."""
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def main():
    evidence = Path(sys.argv[1])
    tree = ET.parse(evidence / 'tomato_picker.urdf')
    alternate = ET.parse(evidence / 'tomato_picker_0deg_lift.urdf')
    packages = {'aubo_description': Path(os.environ['AUBO_SHARE']),
                'tomato_sim_description': Path(os.environ['TOMATO_SHARE'])}
    meshes = tree.findall('.//mesh')
    for mesh in meshes:
        package, _, tail = mesh.get('filename').removeprefix('package://').partition('/')
        assert (packages[package] / tail).is_file(), mesh.attrib
    assert any('aubo_description' in mesh.get('filename') for mesh in meshes)
    slide = tree.find("./joint[@name='machine_slide']")
    assert slide.find('parent').get('link') == 'track_world'
    assert slide.find('child').get('link') == 'robot_carriage'
    assert slide.get('type') == 'prismatic' and slide.find('axis').get('xyz') == '1 0 0'
    assert float(slide.find('origin').get('xyz').split()[2]) == 0.04
    assert [float(slide.find('limit').get(limit)) for limit in ('lower', 'upper')] == [-2.5, 2.5]
    for name in ('left_camera_mount', 'right_camera_mount'):
        path = f"./joint[@name='{name}']/origin"
        default_yaw = float(tree.find(path).get('rpy').split()[2])
        straight_yaw = float(alternate.find(path).get('rpy').split()[2])
        assert abs(abs(default_yaw - straight_yaw) - math.radians(14)) < 1e-9
    assert float(alternate.find("./joint[@name='lift_mount']/origin").get('xyz').split()[2]) == -0.2
    sdf = ET.parse(evidence / 'tomato_picker.sdf')
    anchor = sdf.find(".//joint[@name='track_anchor']")
    assert anchor.findtext('parent') == 'world' and anchor.findtext('child') == 'track_world'
    assert sdf.findtext('./model/self_collide') == '1'
    assert len(sdf.findall(".//sensor[@type='depth']")) == 3
    assert sdf.find(".//plugin[@filename='libgazebo_ros2_control.so']") is not None
    print(f'Checked {len(meshes)} installed mesh references, fixed rail, three RGB-D sensors,'
          ' 0/14 degree mount and 0.2 m lift offset.')


if __name__ == '__main__':
    main()
