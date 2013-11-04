"""
hubsystem.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from math import *
from common import SubComponent
import numpy as np
from zope.interface import implements

#-------------------------------------------------------------------------------

class Hub(): 
    implements(SubComponent)
    ''' Hub class    
          The Hub class is used to represent the hub component of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''
    
    def __init__(self, BladeMass, RotorDiam, BladeNum, hubDiam=0, RatedWindSpeed = 12, RootMoment=0.0, AirDensity=1.225, Solidity=0.065):
        ''' 
        Initializes hub component 
        
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
        diameter : float
          diameter of the hub [m]
        thickness : float
          average thickness of the hub [m]
        '''

        # Sunderland method for calculating hub, pitch and spinner cone masses
        hubloadFact      = 1                                     # default is 3 blade rigid hub (should depend on hub type)
        hubgeomFact      = 1                                     # default is 3 blade (should depend on blade number)
        hubcontFact      = 1.5                                   # default is 1.5 for full span pitch control (should depend on control method)
        hubmatldensity   = 7860.0                               # density of hub material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        hubmatlstress    = 371000000.0                                # allowable stress of hub material (N / m^2)

        if RootMoment == 0.0:
            RootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided
        
        self.mass =50 * hubgeomFact * hubloadFact * hubcontFact * BladeNum * RootMoment * (hubmatldensity / hubmatlstress)
                                                            # mass of hub based on Sunderland model
                                                            # 31.4 adapted to 50 to fit 5 MW data

        # calculate mass properties
        if hubDiam == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(hubDiam)
        self.thickness = self.diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant

        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * RotorDiam)
        cm[1]     = 0.0
        cm[2]     = 0.025 * RotorDiam
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

#-------------------------------------------------------------------------------

class PitchSystem(): 
    implements(SubComponent) 
    '''
     PitchSystem class          
      The PitchSystem class is used to represent the pitch system of a wind turbine. 
      It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
      It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    def __init__(self, BladeMass, RotorDiam, BladeNum, hubDiam=0, RatedWindSpeed = 12, RootMoment=0.0, AirDensity=1.225, Solidity=0.065):
        ''' 
        Initializes pitch system
        
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
        diameter : float
          diameter of the pitch system (should be the same as hub diameter) [m]
        '''

        # Sunderland method for calculating pitch system masses
        pitchmatldensity = 7860.0                             # density of pitch system material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        pitchmatlstress  = 371000000.0                              # allowable stress of hub material (N / m^2)

        if RootMoment == 0.0:
            RootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided

        hubpitchFact      = 1.0                                 # default factor is 1.0 (0.54 for modern designs)
        self.mass =hubpitchFact * (0.22 * BladeMass * BladeNum + 12.6 * BladeNum * RootMoment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model

        # calculate mass properties
        if hubDiam == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(hubDiam)

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * RotorDiam)
        cm[1]     = 0.0
        cm[2]     = 0.025 * RotorDiam
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = self.mass * (self.diameter ** 2) / 4
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

#-------------------------------------------------------------------------------      

class Spinner(): 
    implements(SubComponent) 
    ''' 
       Spinner class
          The SpinnerClass is used to represent the spinner of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    def __init__(self, RotorDiam, hubDiam=0):
        ''' 
        Initializes spinner component 
        
        Parameters
        ----------
        RotorDiam : float
          The wind turbine rotor diameter [m]
        hubDiam : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]
        diameter : float
          diameter of the spinner / nose cone (should be the same as hub diameter) [m]
        thickness : float
          thickness of the spinner / nose cone [m]
        '''

        self.mass =18.5 * RotorDiam + (-520.5)   # spinner mass comes from cost and scaling model

        # calculate mass properties
        if hubDiam == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(hubDiam)
        self.thickness = self.diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant

        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * RotorDiam)
        cm[1]     = 0.0
        cm[2]     = 0.025 * RotorDiam
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

#-------------------------------------------------------------------------------

if __name__ == "__main__":

    pass