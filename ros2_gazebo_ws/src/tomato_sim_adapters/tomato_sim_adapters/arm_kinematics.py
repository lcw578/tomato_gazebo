"""Build a KDL chain directly from the upstream AUBO i5 URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import PyKDL as kdl

JOINTS = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
OEM_HOME = np.deg2rad([10.32, 15.37, 134.05, 117.05, 86.63, 0.0])
BOUND = 3.0543


class ArmKinematics:
    def __init__(self):
        urdf = Path(get_package_share_directory('aubo_description')) / 'urdf/aubo_i5.urdf'
        joints = {element.get('name'): element for element in ET.parse(urdf).getroot().findall('joint')}
        self.chain = kdl.Chain()
        for name in JOINTS:
            element = joints[name]
            origin = element.find('origin')
            frame = kdl.Frame(kdl.Rotation.RPY(*map(float, origin.get('rpy').split())),
                              kdl.Vector(*map(float, origin.get('xyz').split())))
            axis = kdl.Vector(*map(float, element.find('axis').get('xyz').split()))
            joint = kdl.Joint(name, frame.p, frame.M * axis, kdl.Joint.RotAxis)
            self.chain.addSegment(kdl.Segment(element.find('child').get('link'), joint, frame))
        wrist_to_flange = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0.0, 0.0, 0.050952))
        flange_to_cutter = kdl.Frame(kdl.Rotation.RPY(np.pi, 0.0, np.pi / 2), kdl.Vector())
        cutter_to_tcp = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.0049, -0.0044, -0.085))
        self.wrist_to_tcp = wrist_to_flange * flange_to_cutter * cutter_to_tcp
        self.forward = kdl.ChainFkSolverPos_recursive(self.chain)
        self.velocity = kdl.ChainIkSolverVel_pinv(self.chain)
        self.inverse = kdl.ChainIkSolverPos_NR_JL(
            self.chain, self.array([-BOUND] * 6), self.array([BOUND] * 6),
            self.forward, self.velocity, 200, 1e-6)
        self.fallback = kdl.ChainIkSolverPos_LMA(self.chain, 1e-7, 250, 1e-12)

    @staticmethod
    def array(values):
        result = kdl.JntArray(6)
        for index, value in enumerate(values):
            result[index] = float(value)
        return result

    def fk(self, q):
        pose = kdl.Frame()
        if self.forward.JntToCart(self.array(q), pose) < 0:
            raise ValueError('AUBO FK failed')
        return pose

    def fk_tcp(self, q):
        return self.fk(q) * self.wrist_to_tcp

    def ik(self, xyz, rotation, seed, alternatives=False):
        goal = kdl.Frame(rotation, kdl.Vector(*xyz))
        seeds = [np.asarray(seed)]
        if alternatives:
            seeds.extend([OEM_HOME, np.zeros(6)])
            for shoulder in (-1.3, 0.0, 1.3):
                for elbow in (-1.2, 1.2):
                    seeds.append(np.array([shoulder, 0.2, elbow, -0.5, 0.7, 0.0]))
        output = kdl.JntArray(6)
        for q in seeds:
            for solver in (self.inverse, self.fallback):
                solver.CartToJnt(self.array(q), goal, output)
                candidate = np.array([(output[i] + np.pi) % (2 * np.pi) - np.pi for i in range(6)])
                if np.any(np.abs(candidate) > BOUND) or not np.isfinite(candidate).all():
                    continue
                error = kdl.diff(self.fk(candidate), goal)
                if error.vel.Norm() < 0.0002 and error.rot.Norm() < 0.002:
                    return candidate
        raise ValueError(f'No AUBO IK solution for {list(xyz)}')

    def ik_tcp(self, xyz, rotation, seed, alternatives=False):
        goal_tcp = kdl.Frame(rotation, kdl.Vector(*xyz))
        goal_wrist = goal_tcp * self.wrist_to_tcp.Inverse()
        return self.ik(list(goal_wrist.p), goal_wrist.M, seed, alternatives=alternatives)
