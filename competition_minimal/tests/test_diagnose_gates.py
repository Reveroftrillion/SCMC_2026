from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from diagnose_gates import diagnose


class GatesDiagnosticTest(unittest.TestCase):
    def test_actual_configuration_and_order(self):
        lines = diagnose(ROOT)
        self.assertTrue(any('sensor_reference_verified=False' in line for line in lines))
        self.assertTrue(any('drive=False' in line for line in lines))
        events = [line for line in lines if line.startswith('s=')]
        distances = [float(line[2:].split()[0]) for line in events]
        self.assertEqual(distances, sorted(distances))
        self.assertFalse(any('Static start XY not provided' in line for line in events))
        self.assertTrue(any('399.776' in line and 'unverified section' in line for line in events))
        self.assertTrue(any('564.446' in line and 'traffic 4 unverified' in line for line in events))
        self.assertTrue(any('FINISH' in line for line in events))

    def test_enable_control_defaults_to_drive(self):
        text = '\n'.join(diagnose(ROOT, {'drive': 'true', 'status_layout_confirmed': 'true'}))
        self.assertIn('drive=True; enable_control=True; status_layout_confirmed=True', text)
        text = '\n'.join(diagnose(ROOT, {'drive': 'true', 'enable_control': 'false'}))
        self.assertIn('drive=True; enable_control=False', text)
        text = '\n'.join(diagnose(ROOT, {'drive': 'false', 'enable_control': 'true'}))
        self.assertIn('transmit needs all three', text)


if __name__ == '__main__':
    unittest.main()
