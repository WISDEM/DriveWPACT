"""
hubSE.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Assembly
from openmdao.main.datatypes.api import Float, Int, Array
from math import pi

from HubSE_components import Hub, PitchSystem, Spinner, HubSystemAdder
from DriveSE_components import Hub_drive


class HubBase(Assembly):

    # variables
    blade_mass = Float(iotype='in', units='kg', desc='mass of one blade')
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    blade_root_diameter = Float(iotype='in', units='m', desc='blade root diameter')

    # parameters
    blade_number = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(iotype='out', desc='center of mass of the hub relative to tower to in yaw-aligned c.s.')
    I = Array(iotype='out', desc='mass moments of Inertia of hub [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] around its center of mass in yaw-aligned c.s.')

    hub_mass = Float(0.0, iotype='out', units='kg')
    pitch_system_mass = Float(0.0, iotype='out', units='kg')
    spinner_mass = Float(0.0, iotype='out', units='kg')


class HubSE(HubBase):
    '''
       HubSE class
          The HubSE class is used to represent the hub system of a wind turbine.
    '''

    # # parameters
    # hub_diameter = Float(0.0, iotype='in', units='m', desc='hub diameter')


    def configure(self):

        # select components
        self.add('hubSystem', HubSystemAdder())
        self.add('hub', Hub())
        self.add('pitchSystem', PitchSystem())
        self.add('spinner', Spinner())

        # workflow
        self.driver.workflow.add(['hubSystem', 'hub', 'pitchSystem', 'spinner'])

        # connect inputs
        self.connect('blade_mass', ['pitchSystem.blade_mass'])
        self.connect('rotor_bending_moment', ['hub.rotor_bending_moment', 'pitchSystem.rotor_bending_moment'])
        self.connect('blade_number', ['hub.blade_number', 'pitchSystem.blade_number'])
        self.connect('rotor_diameter', ['hub.rotor_diameter', 'pitchSystem.rotor_diameter', 'spinner.rotor_diameter'])
        self.connect('blade_root_diameter', ['hub.hub_diameter', 'pitchSystem.hub_diameter', 'spinner.hub_diameter'])

        # connect components
        self.connect('hub.mass', 'hubSystem.hubMass')
        self.connect('hub.cm', 'hubSystem.hubCM')
        self.connect('hub.I', 'hubSystem.hubI')
        self.connect('pitchSystem.mass', 'hubSystem.pitchMass')
        self.connect('pitchSystem.cm', 'hubSystem.pitchCM')
        self.connect('pitchSystem.I', 'hubSystem.pitchI')
        self.connect('spinner.mass', 'hubSystem.spinnerMass')
        self.connect('spinner.cm', 'hubSystem.spinnerCM')
        self.connect('spinner.I', 'hubSystem.spinnerI')

        # connect outputs
        self.connect('hubSystem.mass', 'mass')
        self.connect('hubSystem.cm', 'cm')
        self.connect('hubSystem.I', 'I')
        self.connect('hub.mass', 'hub_mass')
        self.connect('pitchSystem.mass', 'pitch_system_mass')
        self.connect('spinner.mass', 'spinner_mass')

#-------------------------------------------------------------------------------


class HubSE_drive(HubBase):
    '''
       HubSE class
          The HubSE class is used to represent the hub system of a wind turbine.
    '''

    def configure(self):

        # select components
        self.add('hubSystem', HubSystemAdder())
        self.add('hub', Hub_drive())
        self.add('pitchSystem', PitchSystem())
        self.add('spinner', Spinner())

        # workflow
        self.driver.workflow.add(['hubSystem', 'hub', 'pitchSystem', 'spinner'])

        # connect inputs
        self.connect('blade_mass', ['pitchSystem.blade_mass'])
        self.connect('rotor_bending_moment', ['pitchSystem.rotor_bending_moment'])
        self.connect('blade_number', ['hub.blade_number', 'pitchSystem.blade_number'])
        self.connect('rotor_diameter', ['hub.rotor_diameter', 'pitchSystem.rotor_diameter', 'spinner.rotor_diameter'])
        self.connect('hub.diameter', ['pitchSystem.hub_diameter', 'spinner.hub_diameter'])
        self.connect('blade_root_diameter', 'hub.blade_root_diameter')

        # connect components
        self.connect('hub.mass', 'hubSystem.hubMass')
        self.connect('hub.cm', 'hubSystem.hubCM')
        self.connect('hub.I', 'hubSystem.hubI')
        self.connect('pitchSystem.mass', 'hubSystem.pitchMass')
        self.connect('pitchSystem.cm', 'hubSystem.pitchCM')
        self.connect('pitchSystem.I', 'hubSystem.pitchI')
        self.connect('spinner.mass', 'hubSystem.spinnerMass')
        self.connect('spinner.cm', 'hubSystem.spinnerCM')
        self.connect('spinner.I', 'hubSystem.spinnerI')

        # connect outputs
        self.connect('hubSystem.mass', 'mass')
        self.connect('hubSystem.cm', 'cm')
        self.connect('hubSystem.I', 'I')
        self.connect('hub.mass', 'hub_mass')
        self.connect('pitchSystem.mass', 'pitch_system_mass')
        self.connect('spinner.mass', 'spinner_mass')


#-------------------------------------------------------------------------------

def example():

    # simple test of module

    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    hub = HubSE_drive()
    hub.blade_mass = 17740.0 # kg
    hub.rotor_diameter = 126.0 # m
    hub.blade_number  = 3
    hub.blade_root_diameter   = 3.542
    #AirDensity= 1.225 # kg/(m^3)
    #Solidity  = 0.0517
    #RatedWindSpeed = 11.05 # m/s
    #hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number
    #print hub.rotor_bending_moment
    hub.rotor_bending_moment = 16665000.0 # y-direction
    #hub.rotor_bending_moment = 21529000.0 #combined x-y for single blade

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
    print 'Hub system total {0:8.1f} kg'.format(hub.mass) # 50458.95
    print '    cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print '    I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

def example_80m_redesign():

    # simple test of module

    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    hub = HubSE_drive()
    hub.blade_mass = 16097.0 # kg
    hub.rotor_diameter = 126.0 # m
    hub.blade_number  = 3
    hub.blade_root_diameter   = 3.405
    #AirDensity= 1.225 # kg/(m^3)
    #Solidity  = 0.0517
    #RatedWindSpeed = 11.05 # m/s
    #hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number
    #print hub.rotor_bending_moment
    hub.rotor_bending_moment = 14619000 # y-direction
    #hub.rotor_bending_moment = 21529000.0 #combined x-y for single blade

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
    print 'Hub system total {0:8.1f} kg'.format(hub.mass) # 50458.95
    print '    cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print '    I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])


def example_100m_redesign():

    # simple test of module

    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    hub = HubSE_drive()
    hub.blade_mass = 16423.0 # kg
    hub.rotor_diameter = 126.0 # m
    hub.blade_number  = 3
    hub.blade_root_diameter   = 3.405
    #AirDensity= 1.225 # kg/(m^3)
    #Solidity  = 0.0517
    #RatedWindSpeed = 11.05 # m/s
    #hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number
    #print hub.rotor_bending_moment
    hub.rotor_bending_moment = 14619000 # y-direction
    #hub.rotor_bending_moment = 21529000.0 #combined x-y for single blade

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
    print 'Hub system total {0:8.1f} kg'.format(hub.mass) # 50458.95
    print '    cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print '    I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

def example2():

    # WindPACT 1.5 MW turbine
    print "WindPACT 1.5 MW turbine test"
    hub = HubSE()
    hub.blade_mass = 4470.0
    hub.rotor_diameter = 70.0
    hub.bladeNumer  = 3
    hub.hub_diameter   = 3.0
    AirDensity= 1.225
    Solidity  = 0.065
    RatedWindSpeed = 12.12
    hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

    # GRC 750 kW turbine
    print "windpact 750 kW turbine test"
    hub.blade_mass = 3400.0
    hub.rotor_diameter = 48.2
    hub.bladeNumer = 3
    hub.hub_diameter = 3.0
    AirDensity = 1.225
    Solidity = 0.07 # uknown value
    RatedWindSpeed = 16.0
    hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

    # Windpact / Alstom 3.0 MW
    print "windpact / Alstom 3.0 MW"
    hub.blade_mass = 13363.0 # from CSM - not included in windpact/alstom data
    hub.rotor_diameter = 100.8 # alstom
    hub.bladeNumer = 3
    hub.hub_diameter = 0.0
    AirDensity = 1.225
    Solidity = 0.06 # unknown value
    RatedWindSpeed = 12.0 # expected to be closer to 1.5 and 5 MW
    hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

def example_100m_redesign_ideal():
    # simple test of module
    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    hub = HubSE_drive()
    hub.blade_mass = 16097.0 # kg
    hub.rotor_diameter = 126.0 # m
    hub.blade_number  = 3
    hub.blade_root_diameter   = 3.405
    #AirDensity= 1.225 # kg/(m^3)
    #Solidity  = 0.0517
    #RatedWindSpeed = 11.05 # m/s
    #hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number
    #print hub.rotor_bending_moment
    hub.rotor_bending_moment = 14619000 # y-direction
    #hub.rotor_bending_moment = 21529000.0 #combined x-y for single blade

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
    print 'Hub system total {0:8.1f} kg'.format(hub.mass) # 50458.95
    print '    cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print '    I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

if __name__ == "__main__":

    example()
    example_80m_redesign()
    example_100m_redesign()
    example_100m_redesign_ideal()
    #example2()
