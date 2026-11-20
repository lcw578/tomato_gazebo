"""Verify that URDF conversion leaves the blade sensor attached to a collision."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


class BladeContactSensorTest(unittest.TestCase):
    def test_sensor_references_an_existing_sdf_collision(self):
        source = Path(__file__).resolve().parents[1] / 'src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro'
        urdf = subprocess.check_output(['xacro', str(source)], text=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.urdf'
            path.write_text(urdf)
            sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(path)], text=True))
        finger = sdf.find("model/link[@name='left_finger_link']")
        sensor = finger.find("sensor[@name='mujoco_contact_blade_sensor']")
        self.assertIn(sensor.findtext('contact/collision'),
                      [collision.get('name') for collision in finger.findall('collision')])


if __name__ == '__main__':
    unittest.main()
