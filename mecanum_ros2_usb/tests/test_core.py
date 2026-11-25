import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_core import to_pwm, select_command, feedback

class CoreTest(unittest.TestCase):
    def pwm(self, x,y,z):
        return to_pwm(x,y,z,.05,.15,.13,5.,100)
    def test_basis(self):
        self.assertEqual(self.pwm(.1,0,0),[10]*4)
        self.assertEqual(self.pwm(0,.1,0),[-10,10,10,-10])
        self.assertEqual(self.pwm(0,0,.5),[-14,14,-14,14])
    def test_stop_and_symmetry(self):
        self.assertEqual(self.pwm(0,0,0),[0]*4)
        self.assertEqual(self.pwm(-.1,-.1,-.2),[-v for v in self.pwm(.1,.1,.2)])
    def test_scaling(self):
        self.assertEqual(self.pwm(3,1,0),[50,100,100,50])
    def test_invalid(self):
        for x in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.pwm(x,0,0)
        with self.assertRaises(ValueError):to_pwm(0,0,0,0,.1,.1,5,100)
    def test_timeout(self):
        c=(.1,0,0)
        self.assertEqual(select_command(c,1,1.29,.3,True),c)
        self.assertEqual(select_command(c,1,1.31,.3,True),(0,0,0))
        self.assertEqual(select_command(c,1,1.1,.3,False),(0,0,0))
        self.assertEqual(select_command(c,None,1.1,.3,True),(0,0,0))
    def test_feedback(self):
        self.assertEqual(feedback('T,100,2,10,10,10,10,0'),[100,2,10,10,10,10,0])
        for s in ['T,0,0,161,0,0,0,0','T,0,0,0,0,0,0,2','E,1,2','T,x,0,0,0,0,0,0']:
            self.assertIsNone(feedback(s))

if __name__=='__main__':unittest.main()
