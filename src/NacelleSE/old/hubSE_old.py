"""
hubsystem.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from math import *
from common import SubComponent
import numpy as np
from zope.interface import implements

from hub_components import Hub, PitchSystem, Spinner

#-------------------------------------------------------------------------------

class HubSystem(): 
    implements(SubComponent) 
    ''' 
       HubSystem class
          The HubSystem class is used to represent the hub system of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''
    
    def __init__(self, BladeMass, RotorDiam, BladeNum, hubDiam=0, RatedWindSpeed = 12, RootMoment=0.0, AirDensity=1.225, Solidity=0.065):
        ''' 
        Initializes hub system component 
        
        Parameters
        ----------
        RotorDiam : float
          The wind turbine rotor diameter [m]
        BladeMass : float
          The wind turbine individual blade mass [kg]
        BladeNum : int
          Number of wind turbine rotor blades
        hubDiam : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]
        RatedWindSpeed : float
          wind speed for rated power (default == 12 if not provided) [m/s]
        RootMoment : float
          maximum flap-wise root moment for the blade (if == 0, then it is set within compute method) [Nm]
        AirDensity : float
          air density at hub height (default == 1.225 kg / m^3 if not provided) [kg / m^3]
        Solidity : float
          solidity of the rotor (default == 0.065 if not provided)
        pitchSys : PitchSystem
          the pitch system hub component
        hub : Hub
          the hub sub component
        spinner : Spinner
          the spinner / nose cone sub component
        '''

        self.hub = Hub(BladeMass, RotorDiam, BladeNum, hubDiam, RatedWindSpeed, RootMoment, AirDensity, Solidity)
        self.pitchSystem = PitchSystem(BladeMass, RotorDiam, BladeNum, hubDiam, RatedWindSpeed, RootMoment, AirDensity, Solidity)
        self.spinner = Spinner(RotorDiam, hubDiam)

        self.mass =self.hub.mass + self.pitchSystem.mass + self.spinner.mass

        # calculate mass properties
        # calculation of mass center and moments of inertia
        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass
            cm[i] = (self.hub.mass * self.hub.cm[i] + self.pitchSystem.mass * self.pitchSystem.cm[i] + \
                    self.spinner.mass * self.spinner.cm[i] ) / (self.hub.mass + self.pitchSystem.mass + self.spinner.mass)
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM
            I[i]  =  self.hub.I[i] + self.pitchSystem.I[i] + self.spinner.I[i]
            # translate to hub system CM using parallel axis theorem
            for j in (range(0,3)): 
                if i != j:
                    I[i] +=  (self.hub.mass * (self.hub.cm[i] - self.cm[i]) ** 2) + \
                                  (self.pitchSystem.mass * (self.pitchSystem.cm[i] - self.cm[i]) ** 2) + \
                                  (self.spinner.mass * (self.spinner.cm[i] - self.cm[i]) ** 2)
        self.I = I

#-------------------------------------------------------------------------------

def example():
  
    # simple test of module

    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    BladeMass = 17740.0 # kg
    RotorDiam = 126.0 # m
    BladeNum  = 3
    hubDiam   = 3.0 # m
    RootMoment= 0.0 # Nm
    AirDensity= 1.225 # kg/(m^3)
    Solidity  = 0.0517 
    RatedWindSpeed = 11.05 # m/s
    hub = HubSystem(BladeMass, RotorDiam, BladeNum, hubDiam, RatedWindSpeed, RootMoment, AirDensity, Solidity)

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
    print 'Hub system total {0:8.1f} kg'.format(hub.mass) # 50458.95
    print '    cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print '    I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

def example2():

    # simple test of module

    # WindPACT 1.5 MW turbine
    print "WindPACT 1.5 MW turbine test"
    BladeMass = 4470.0
    RotorDiam = 70.0
    BladeNum  = 3
    hubDiam   = 3.0
    RootMoment= 0.0
    AirDensity= 1.225
    Solidity  = 0.065
    RatedWindSpeed = 12.12
    hub = HubSystem(BladeMass, RotorDiam, BladeNum, hubDiam, RatedWindSpeed, RootMoment, AirDensity, Solidity)

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

    # GRC 750 kW turbine
    print "windpact 750 kW turbine test"
    BladeMass = 3400.0
    RotorDiam = 48.2
    BladeNum = 3
    hubDiam = 3.0
    RootMoment = 0.0
    AirDensity = 1.225
    Solidity = 0.07 # uknown value
    RatedWindSpeed = 16.0
    hub = HubSystem(BladeMass, RotorDiam, BladeNum, hubDiam, RatedWindSpeed, RootMoment, AirDensity, Solidity)

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

    # Windpact / Alstom 3.0 MW
    print "windpact / Alstom 3.0 MW"
    BladeMass = 13363.0 # from CSM - not included in windpact/alstom data
    RotorDiam = 100.8 # alstom
    BladeNum = 3
    hubDiam = 0.0
    RootMoment = 0.0
    AirDensity = 1.225
    Solidity = 0.06 # unknown value
    RatedWindSpeed = 12.0 # expected to be closer to 1.5 and 5 MW
    hub = HubSystem(BladeMass, RotorDiam, BladeNum, hubDiam, RatedWindSpeed, RootMoment, AirDensity, Solidity)   
    
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