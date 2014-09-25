
#!/usr/bin/env python
# encoding: utf-8
"""
test_Turbine_CostsSE.py

Created by Katherine Dykes on 2014-01-07.
Copyright (c) NREL. All rights reserved.
"""

import unittest
import numpy as np
from math import pi
from commonse.utilities import check_gradient_unit_test

from drivewpact.hub import Hub, PitchSystem, Spinner, HubSystemAdder, HubWPACT
from drivewpact.drive import LowSpeedShaft, MainBearing, SecondBearing, Gearbox, HighSpeedSide, Generator, Bedplate, AboveYawMassAdder, YawSystem, NacelleSystemAdder, DriveWPACT

# Hub Components
class Test_Hub(unittest.TestCase):

    def setUp(self):

        self.hub = Hub()

        self.hub.rotor_diameter = 126.0 # m
        self.hub.blade_number  = 3
        self.hub.hub_diameter   = 3.542
        AirDensity= 1.225 # kg/(m^3)
        Solidity  = 0.0517
        RatedWindSpeed = 11.05 # m/s
        self.hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (self.hub.rotor_diameter ** 3)) / self.hub.blade_number

    def test_functionality(self):
        
        self.hub.run()
        
        self.assertEqual(round(self.hub.mass,1), 29536.2)

    def test_gradient(self):

        check_gradient_unit_test(self, self.hub,display=False)

class Test_PitchSystem(unittest.TestCase):

    def setUp(self):

        self.pitch = PitchSystem()

        self.pitch.blade_mass = 17740.0 # kg
        self.pitch.rotor_diameter = 126.0 # m
        self.pitch.blade_number  = 3
        self.pitch.hub_diameter   = 3.542
        AirDensity= 1.225 # kg/(m^3)
        Solidity  = 0.0517
        RatedWindSpeed = 11.05 # m/s
        self.pitch.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (self.pitch.rotor_diameter ** 3)) / self.pitch.blade_number

    def test_functionality(self):
        
        self.pitch.run()
        
        self.assertEqual(round(self.pitch.mass,1), 13362.4)

    def test_gradient(self):

        check_gradient_unit_test(self, self.pitch,display=False)

class Test_Spinner(unittest.TestCase):

    def setUp(self):

        self.spinner = Spinner()

        self.spinner.rotor_diameter = 126.0 # m
        self.spinner.hub_diameter   = 3.542

    def test_functionality(self):
        
        self.spinner.run()
        
        self.assertEqual(round(self.spinner.mass,1), 1810.5)

    def test_gradient(self):

        check_gradient_unit_test(self, self.spinner,display=False)

class Test_HubSystemAdder(unittest.TestCase):

    def setUp(self):

        self.hub_system = HubSystemAdder()
        self.hub_system.hub_mass = 10000.
        self.hub_system.pitch_system_mass = 10000.
        self.hub_system.spinner_mass = 1000.
        self.hub_system.hub_cm = [-2.,1.0, 1.0]
        self.hub_system.pitch_cm = [-2., 1.0, 1.0]
        self.hub_system.spinner_cm = [-2., 1.0, 1.0]
        self.hub_system.hub_I = [1000., 1000., 1000.]
        self.hub_system.pitch_I = [1000., 1000., 1000.]
        self.hub_system.spinner_I = [1000., 1000., 1000.]

    def test_functionality(self):
        
        self.hub_system.run()
        
        self.assertEqual(round(self.hub_system.hub_system_mass,1), 21000.0)

    def test_gradient(self):

        check_gradient_unit_test(self, self.hub_system,tol=5e-6, display=False)

class Test_HubWPACT(unittest.TestCase):

    def setUp(self):

        self.hub = HubWPACT()

        self.hub.blade_mass = 17740.0 # kg
        self.hub.rotor_diameter = 126.0 # m
        self.hub.blade_number  = 3
        self.hub.blade_root_diameter   = 3.542
        AirDensity= 1.225 # kg/(m^3)
        Solidity  = 0.0517
        RatedWindSpeed = 11.05 # m/s
        self.hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (self.hub.rotor_diameter ** 3)) / self.hub.blade_number

    def test_functionality(self):
        
        self.hub.run()
        
        self.assertEqual(round(self.hub.hub_system_mass,1), 44709.1)


# Nacelle Components

class Test_LowSpeedShaft(unittest.TestCase):

    def setUp(self):

        self.lss = LowSpeedShaft()

        self.lss.rotor_diameter = 126. # rotor diameter [m]
        rotor_speed = 12.1
        DrivetrainEfficiency = 0.95
        machine_rating = 5000.0
        self.lss.rotor_torque = 1.5 * (machine_rating * 1000. / DrivetrainEfficiency) / (rotor_speed * (pi / 30.))
        self.lss.rotor_mass = 142585.75 # rotor mass [kg]

    def test_functionality(self):
        
        self.lss.run()
        
        self.assertEqual(round(self.lss.mass,1), 42381.5)

    def test_gradient(self):

        check_gradient_unit_test(self, self.lss,display=False)

class Test_MainBearing(unittest.TestCase):

    def setUp(self):

        self.mb = MainBearing()

        self.mb.lss_design_torque = 18691533.1165
        self.mb.lss_diameter = 1.2115
        self.mb.lss_mass = 42381.5447
        self.mb.rotor_speed = 12.1
        self.mb.rotor_diameter = 126.0

    def test_functionality(self):
        
        self.mb.run()
        
        self.assertEqual(round(self.mb.mass,1), 7348.5)

    def test_gradient(self):

        check_gradient_unit_test(self, self.mb,display=False)

class Test_SecondBearing(unittest.TestCase):

    def setUp(self):

        self.sb = MainBearing()

        self.sb.lss_design_torque = 18691533.1165
        self.sb.lss_diameter = 1.2115
        self.sb.lss_mass = 42381.5447
        self.sb.rotor_speed = 12.1
        self.sb.rotor_diameter = 126.0

    def test_functionality(self):
        
        self.sb.run()
        
        self.assertEqual(round(self.sb.mass,1), 7348.5)

    def test_gradient(self):

        check_gradient_unit_test(self, self.sb,display=False)

class Test_Gearbox(unittest.TestCase):

    def setUp(self):

        self.gbx = Gearbox()

        self.gbx.drivetrain_design = 'geared' # geared 3-stage Gearbox with induction generator machine
        self.gbx.machine_rating = 5000. # machine rating [kW]
        self.gbx.gear_ratio = 96.76
        self.gbx.gear_configuration = 'eep'
        self.gbx.bevel = 0

        airdensity = 1.225 # air density [kg / m^3]
        MaxTipSpeed = 80. # max tip speed [m/s]
        self.gbx.rotor_diameter = 126. # rotor diameter [m]
        self.gbx.rotor_speed = 12.1
        DrivetrainEfficiency = 0.95
        self.gbx.rotor_torque = 1.5 * (self.gbx.machine_rating * 1000. / DrivetrainEfficiency) / (self.gbx.rotor_speed * (np.pi / 30.))

    def test_functionality(self):
        
        self.gbx.run()
        
        self.assertEqual(round(self.gbx.mass,1), 48664.7)

    def test_gradient(self):

        check_gradient_unit_test(self, self.gbx,display=False)

class Test_HighSpeedSide(unittest.TestCase):

    def setUp(self):

        self.hss = HighSpeedSide()

        self.hss.gear_ratio = 96.76
        self.hss.rotor_diameter = 126. # rotor diameter [m]
        rotor_speed = 12.1
        DrivetrainEfficiency = 0.95
        machine_rating = 5000.0
        self.hss.rotor_torque = 1.5 * (machine_rating * 1000. / DrivetrainEfficiency) / (rotor_speed * (pi / 30.))
        self.hss.lss_diameter = 1.2115

    def test_functionality(self):
        
        self.hss.run()
        
        self.assertEqual(round(self.hss.mass,1), 2414.7)

    def test_gradient(self):

        check_gradient_unit_test(self, self.hss,display=False)

class Test_Generator(unittest.TestCase):

    def setUp(self):

        self.gen = Generator()

        self.gen.gear_ratio = 96.76
        self.gen.rotor_diameter = 126. # rotor diameter [m]
        self.gen.machine_rating = 5000.
        self.gen.drivetrain_design = 'geared'

    def test_functionality(self):
        
        self.gen.run()
        
        self.assertEqual(round(self.gen.mass,1), 16699.9)

    def test_gradient(self):

        check_gradient_unit_test(self, self.gen,display=False)

class Test_Bedplate(unittest.TestCase):

    def setUp(self):

        self.bpl = Bedplate()

        self.bpl.drivetrain_design = 'geared'

        self.bpl.rotor_diameter = 126.
        rotor_speed = 12.1
        DrivetrainEfficiency = 0.95
        machine_rating = 5000.
        self.bpl.rotor_torque = 1.5 * (machine_rating * 1000. / DrivetrainEfficiency) / (rotor_speed * (pi / 30.))
        self.bpl.rotor_thrust = 2.5448e5
        self.bpl.rotor_mass = 142585.75
        self.bpl.tower_top_diameter = 3.78 

    def test_functionality(self):
        
        self.bpl.run()
        
        self.assertEqual(round(self.bpl.mass,1), 108512.5)

    def test_gradient(self):

        check_gradient_unit_test(self, self.bpl, display=False)

class Test_AboveYawMassAdder(unittest.TestCase):

    def setUp(self):

        self.yawadder = AboveYawMassAdder()

        self.yawadder.machine_rating = 5000.0
        self.yawadder.lss_mass = 42381.5
        self.yawadder.main_bearing_mass = 14696.2/2.
        self.yawadder.second_bearing_mass = 14696.2/2.
        self.yawadder.gearbox_mass = 48664.7
        self.yawadder.hss_mass = 2414.7
        self.yawadder.generator_mass = 16699.9
        self.yawadder.bedplate_mass = 108512.5
        self.yawadder.bedplate_length = 10.4006
        self.yawadder.bedplate_width = 5.20032
        self.yawadder.crane = True

    def test_functionality(self):
        
        self.yawadder.run()
        
        self.assertEqual(round(self.yawadder.above_yaw_mass,1), 259430.9)

    def test_gradient(self):

        check_gradient_unit_test(self, self.yawadder,display=False)

class Test_YawSystem(unittest.TestCase):

    def setUp(self):

        self.yaw = YawSystem()

        self.yaw.rotor_diameter = 126. # rotor diameter [m]
        self.yaw.rotor_thrust = 2.5448e5
        self.yaw.tower_top_diameter = 3.78 # tower top diameter [m]
        self.yaw.above_yaw_mass = 259430.9

    def test_functionality(self):
        
        self.yaw.run()
        
        self.assertEqual(round(self.yaw.mass,1), 13789.0)

    def test_gradient(self):

        check_gradient_unit_test(self, self.yaw, display=False)


class Test_NacelleSystemAdder(unittest.TestCase):

    def setUp(self):

        self.nac = NacelleSystemAdder()

    def test_functionality(self):

        self.nac.above_yaw_mass = 259430.9
        self.nac.yawMass = 13789.0
        self.nac.machine_rating = 5000.0
        self.nac.lss_mass = 42381.5
        self.nac.main_bearing_mass = 14696.2/2.
        self.nac.second_bearing_mass = 14696.2/2.
        self.nac.gearbox_mass = 48664.7
        self.nac.hss_mass = 2414.7
        self.nac.generator_mass = 16699.9
        self.nac.bedplate_mass = 108512.5
        self.nac.mainframe_mass = 125076.5
        self.nac.lss_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.main_bearing_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.second_bearing_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.gearbox_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.hss_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.generator_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.bedplate_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.lss_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.main_bearing_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.second_bearing_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.gearbox_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.hss_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.generator_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.bedplate_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])

        self.nac.run()

        self.assertEqual(round(self.nac.nacelle_mass,1), 273219.9)

    def test_gradient(self):

        self.nac.above_yaw_mass = np.random.rand(1)[0]  # 5000.
        self.nac.yawMass = np.random.rand(1)[0]  # 5000.
        self.nac.lss_mass = np.random.rand(1)[0]  # 5000.
        self.nac.main_bearing_mass = np.random.rand(1)[0]  # 5000.
        self.nac.second_bearing_mass = np.random.rand(1)[0]  # 5000.
        self.nac.gearbox_mass = np.random.rand(1)[0]  # 5000.
        self.nac.hss_mass = np.random.rand(1)[0]  # 5000.
        self.nac.generator_mass = np.random.rand(1)[0]  # 5000.
        self.nac.bedplate_mass = np.random.rand(1)[0]  # 5000.
        self.nac.mainframe_mass = np.random.rand(1)[0]  # 6000.
        self.nac.lss_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.main_bearing_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.second_bearing_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.gearbox_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.hss_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.generator_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.bedplate_cm = np.random.rand(3)  # np.array([-2.0, 1.0, 1.0])
        self.nac.lss_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.main_bearing_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.second_bearing_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.gearbox_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.hss_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.generator_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])
        self.nac.bedplate_I = np.random.rand(3)  # np.array([1000., 1000., 1000.])

        check_gradient_unit_test(self, self.nac, tol=1e-5, display=False)

class Test_Nacelle(unittest.TestCase):

    def setUp(self):

        self.nace = DriveWPACT()
    
        self.nace.rotor_diameter = 126.0 # m
        self.nace.rotor_speed = 12.1 # rpm m/s
        self.nace.machine_rating = 5000.0
        DrivetrainEfficiency = 0.95
        self.nace.rotor_torque =  1.5 * (self.nace.machine_rating * 1000. / DrivetrainEfficiency) / (self.nace.rotor_speed * (pi / 30.)) # 6.35e6 #4365248.74 # Nm
        self.nace.rotor_thrust = 2.5448e5
        self.nace.rotor_mass = 142585.75 #kg
    
        # NREL 5 MW Drivetrain variables
        self.nace.drivetrain_design = 'geared' # geared 3-stage Gearbox with induction generator machine
        self.nace.gear_ratio = 96.76 # 97:1 as listed in the 5 MW reference document
        self.nace.gear_configuration = 'eep' # epicyclic-epicyclic-parallel
        self.nace.bevel = 0 # no bevel stage
        self.nace.crane = True # onboard crane present
    
        # NREL 5 MW Tower Variables
        self.nace.tower_top_diameter = 3.78 # m

    def test_functionality(self):
        
        self.nace.run()
        
        self.assertEqual(round(self.nace.nacelle_mass,1), 273219.9)

if __name__ == "__main__":

    unittest.main()

