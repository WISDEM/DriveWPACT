"""
hubSE.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component, Assembly
from openmdao.main.datatypes.api import Array, Float, Bool, Int
from math import pi

from hubSE_components import Hub, PitchSystem, Spinner, HubSystemAdder

class HubSE(Assembly):
    ''' 
       HubSE class
          The HubSE class is used to represent the hub system of a wind turbine.             
    '''

    # variables
    bladeMass = Float(iotype='in', units='kg', desc='mass of one blade')
    rotorBendingMoment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # parameters
    hubDiameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    bladeNumber = Int(3, iotype='in', desc='number of turbine blades')

    def configure(self):

        # select components
        self.add('hubSystem', HubSystemAdder())
        self.add('hub', Hub())
        self.add('pitchSystem', PitchSystem())
        self.add('spinner', Spinner())
        
        # workflow
        self.driver.workflow.add(['hubSystem','hub', 'pitchSystem', 'spinner'])
        
        # connect inputs
        self.connect('bladeMass', ['pitchSystem.bladeMass'])
        self.connect('rotorBendingMoment', ['hub.rotorBendingMoment', 'pitchSystem.rotorBendingMoment'])
        self.connect('bladeNumber', ['hub.bladeNumber', 'pitchSystem.bladeNumber'])
        self.connect('rotorDiameter', ['hub.rotorDiameter', 'pitchSystem.rotorDiameter', 'spinner.rotorDiameter'])
        self.connect('hubDiameter', ['hub.hubDiameter', 'pitchSystem.hubDiameter', 'spinner.hubDiameter'])
        
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
        
        # create passthroughs
        self.create_passthrough('hubSystem.mass')
        self.create_passthrough('hubSystem.cm')
        self.create_passthrough('hubSystem.I')

#-------------------------------------------------------------------------------

class HubSE_drive(Assembly):
    ''' 
       HubSE class
          The HubSE class is used to represent the hub system of a wind turbine.             
    '''

    # variables
    #bladeMass = Float(iotype='in', units='kg', desc='mass of one blade')
    rotorBendingMoment = Float(iotype='in', units='N*m', desc='maximum aerodynamic bending moment')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    bladeRootDiameter = Float(iotype='in', units='m', desc='blade root diameter')
    
    # parameters
    #hubDiameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    bladeNumber = Int(3, iotype='in', desc='number of turbine blades')

    def configure(self):

        '''# select components
        self.add('hubSystem', HubSystemAdder())
        self.add('hub', Hub())
        self.add('pitchSystem', PitchSystem())
        self.add('spinner', Spinner())
        
        # workflow
        self.driver.workflow.add(['hubSystem','hub', 'pitchSystem', 'spinner'])
        
        # connect inputs
        self.connect('bladeMass', ['pitchSystem.bladeMass'])
        self.connect('rotorBendingMoment', ['hub.rotorBendingMoment', 'pitchSystem.rotorBendingMoment'])
        self.connect('bladeNumber', ['hub.bladeNumber', 'pitchSystem.bladeNumber'])
        self.connect('rotorDiameter', ['hub.rotorDiameter', 'pitchSystem.rotorDiameter', 'spinner.rotorDiameter'])
        self.connect('hubDiameter', ['hub.hubDiameter', 'pitchSystem.hubDiameter', 'spinner.hubDiameter'])
        
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
        
        # create passthroughs
        self.create_passthrough('hubSystem.mass')
        self.create_passthrough('hubSystem.cm')
        self.create_passthrough('hubSystem.I')'''


#-------------------------------------------------------------------------------

def example():
  
    # simple test of module

    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    hub = HubSE()
    hub.bladeMass = 17740.0 # kg
    hub.rotorDiameter = 126.0 # m
    hub.bladeNumber  = 3
    hub.hubDiameter   = 3.0 # m
    AirDensity= 1.225 # kg/(m^3)
    Solidity  = 0.0517 
    RatedWindSpeed = 11.05 # m/s
    hub.rotorBendingMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber
    
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
    hub.bladeMass = 4470.0
    hub.rotorDiameter = 70.0
    hub.bladeNumer  = 3
    hub.hubDiameter   = 3.0
    AirDensity= 1.225
    Solidity  = 0.065
    RatedWindSpeed = 12.12
    hub.rotorBendingMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber

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
    hub.bladeMass = 3400.0
    hub.rotorDiameter = 48.2
    hub.bladeNumer = 3
    hub.hubDiameter = 3.0
    AirDensity = 1.225
    Solidity = 0.07 # uknown value
    RatedWindSpeed = 16.0
    hub.rotorBendingMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber

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
    hub.bladeMass = 13363.0 # from CSM - not included in windpact/alstom data
    hub.rotorDiameter = 100.8 # alstom
    hub.bladeNumer = 3
    hub.hubDiameter = 0.0
    AirDensity = 1.225
    Solidity = 0.06 # unknown value
    RatedWindSpeed = 12.0 # expected to be closer to 1.5 and 5 MW
    hub.rotorBendingMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber 

    hub.run()
    
    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

if __name__ == "__main__":

    example()
    
    example2()
