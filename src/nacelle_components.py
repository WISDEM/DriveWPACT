"""
nacelle.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from math import *
from common import SubComponent
import numpy as np
from zope.interface import implements

# -------------------------------------------------

class LowSpeedShaft():
    implements(SubComponent)
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''

    def __init__(self, RotorDiam, RotorMass, RotorTorque):
        ''' 
        Initializes low speed shaft component 
        
        Parameters
        ----------
        RotorDiam : float
          The wind turbine rotor diameter [m]
        RotorMass : float
          The wind turbine rotor mass [kg]
        RotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        designTQ : float
          Design torque for the low speed shaft based on input torque from the rotor at rated speed accounting for drivetrain losses - multiplied by a safety factor
        designBL : float
          Design bending load based on low speed shaft based on rotor mass
        length : float
          Low Speed Shaft length [m]
        diameter : float
          Low Speed shaft outer diameter [m]
        '''

        self.designTQ = 0.00
        self.designBL = 0.00

        # compute masses, dimensions and cost
        ioratio   = 0.100                                    # constant value for inner/outer diameter ratio (should depend on LSS type)                                                  
        hollow    = 1/(1-(ioratio)**4)                    # hollowness factor based on diameter ratio

        TQsafety       = 3.0                                    # safety factor for design torque applied to rotor torque
        self.designTQ =(TQsafety * RotorTorque)            # LSS design torque [Nm]

        lenFact        = 0.03                                   # constant value for length as a function of rotor diameter (should depend on LSS type) 
        self.length = (lenFact * RotorDiam )              # LSS shaft length [m]                                                                  
        maFact         = 5                                 # moment arm factor from shaft lenght (should depend on shaft type)
        mmtArm    = self.length / maFact            # LSS moment arm [m] - from hub to first main bearing
        BLsafety       = 1.25                                   # saftey factor on bending load
        g              = 9.81                              # gravitational constant [m / s^2]
        self.designBL  = BLsafety * g * RotorMass          # LSS design bending load [N]                                                  
        bendMom   = self.designBL * mmtArm       # LSS design bending moment [Nm]

        yieldst        = 371000000.0                             # BS1503-622 yield stress [Pa] (should be adjusted depending on material type)
        endurstsp      = 309000000.0                             # BS1503-625 specimen endurance limit [Pa] (should be adjusted depending on material type)
        endurFact      = 0.23                                    # factor for specimen to component endurance limit 
                                                                # (0.75 surface condition * 0.65 size * 0.52 reliability * 1 temperature * 0.91 stress concentration)
        endurst        = endurstsp * endurFact                   # endurance limit [Pa] for LSS
        SOsafety       = 3.25                               # Soderberg Line approach factor for safety 
        self.diameter = (((32/pi)*hollow*SOsafety*((self.designTQ / yieldst)**2+(bendMom/endurst)**2)**(0.5))**(1./3.))                               
                                                            # outer diameter [m] computed by Westinghouse Code Formula based on Soderberg Line approach to fatigue design
        inDiam    = self.diameter * ioratio            # inner diameter [m]

        
        massFact       = 1.25                                    # mass weight factor (should depend on LSS/drivetrain type, currently from windpact modifications to account for flange weight)                                                                       
        steeldens      = 7860                                    # steel density [kg / m^3]

        self.mass = (massFact*(pi/4)*(self.diameter**2-inDiam**2)*self.length*steeldens)      # mass of LSS [kg]  

        # calculate mass properties        
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * RotorDiam            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * RotorDiam                      
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (inDiam ** 2 + self.diameter ** 2) / 8
        I[1]  = self.mass * (inDiam ** 2 + self.diameter ** 2 + (4 / 3) * (self.length ** 2)) / 16
        I[2]  = I[1]
        self.I = I

#-------------------------------------------------------------------------------

class Bearing():
	  implements(SubComponent)
	  '''
	     Bearing class for a generic bearing
	  '''

class MainBearings():
    implements(SubComponent) 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    
    def __init__(self,lss, RotorSpeed, RotorDiam):
        ''' Initializes main bearings component 

        Parameters
        ----------
        lss : LowSpeedShaft object
          The low speed shaft object of a wind turbine drivetrain
        RotorSpeed : float
          Speed of the rotor at rated power [rpm]
        RotorDiam : float
          The wind turbine rotor diameter [m]
        inDiam : float
          inner diameter of bearings - equivalent to low speed shaft outer diameter
        '''

        self.mainBearing = Bearing()
        self.secondBearing = Bearing()
        self.inDiam     = 0.0
      
        # compute masses, dimensions and cost
        g = 9.81                                           # gravitational constant [m / s^2]
        design1SL = (4.0 / 3.0) * lss.designTQ + lss.mass * (g / 2.0)
                                                           # front bearing static design load [N] based on default equation (should depend on LSS type)
        design2SL = (1.0 / 3.0) * lss.designTQ - lss.mass * (g / 2.0)
                                                           # rear bearing static design load [N] based on default equation (should depend on LSS type)
        design1DL = 2.29 * design1SL * (RotorSpeed ** 0.3)
                                                           # front bearing dynamic design load [N]
        design2DL = 2.29 * design2SL * (RotorSpeed ** 0.3)
                                                           # rear bearing dynamic design load [N]

        ratingDL  = 17.96 * ((lss.diameter * 1000.0) ** 1.9752)  # basic dynamic load rating for a bearing given inside diameter based on catalogue regression

        massFact  = 0.25                                 # bearing weight factor (should depend on drivetrain type) - using to adjust data closer to cost and scaling model estimates

        if (design1DL < ratingDL):
            b1mass = massFact * (26.13 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.77)
                                                           # bearing mass [kg] for single row bearing (based on catalogue data regression)
            h1mass = massFact * (67.44 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.64)
                                                           # bearing housing mass [kg] for single row bearing (based on catalogue data regression) 
        else:
            b1mass = massFact * 1.7 * (26.13 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.77)
                                                          # bearing mass [kg] for double row bearing (based on catalogue data regression) 
            h1mass = massFact * 1.5 * (67.44 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.64)
                                                          # bearing housing mass [kg] for double row bearing (based on catalogue data regression) 

        if (design2DL < ratingDL):
            b2mass = massFact * (26.13 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.77)
                                                           # bearing mass [kg] for single row bearing (based on catalogue data regression)
            h2mass = massFact * (67.44 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.64)
                                                           # bearing housing mass [kg] for single row bearing (based on catalogue data regression) 
        else:
            b2mass = massFact * 1.7 * (26.13 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.77)
                                                          # bearing mass [kg] for double row bearing (based on catalogue data regression) 
            h2mass = massFact * 1.5 * (67.44 * (10 ** (-6))) * ((lss.diameter * 1000.0) ** 2.64)
                                                          # bearing housing mass [kg] for double row bearing (based on catalogue data regression) 

        self.mainBearing.mass = b1mass + h1mass
        self.secondBearing.mass = b2mass + h2mass

        self.mass = (self.mainBearing.mass + self.secondBearing.mass)

        # calculate mass properties
        inDiam  = lss.diameter
        self.depth = (inDiam * 1.5)

        cmMB = np.array([0.0,0.0,0.0])
        cmMB = ([- (0.035 * RotorDiam), 0.0, 0.025 * RotorDiam])
        self.mainBearing.cm = cmMB

        cmSB = np.array([0.0,0.0,0.0])
        cmSB = ([- (0.01 * RotorDiam), 0.0, 0.025 * RotorDiam])
        self.secondBearing.cm = cmSB

        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass
            cm[i] = (self.mainBearing.mass * self.mainBearing.cm[i] + self.secondBearing.mass * self.secondBearing.cm[i]) \
                      / (self.mainBearing.mass + self.secondBearing.mass)
        self.cm = cm
       
        self.b1I0 = (b1mass * inDiam ** 2 ) / 4 + (h1mass * self.depth ** 2) / 4
        self.mainBearing.I = ([self.b1I0, self.b1I0 / 2, self.b1I0 / 2]) 

        self.b2I0  = (b2mass * inDiam ** 2 ) / 4 + (h2mass * self.depth ** 2) / 4
        self.secondBearing.I = ([self.b2I0, self.b2I0 / 2, self.b2I0 / 2])

        I = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM
            I[i]  =  self.mainBearing.I[i] + self.secondBearing.I[i]
            # translate to nacelle CM using parallel axis theorem
            for j in (range(0,3)): 
                if i != j:
                    I[i] +=  (self.mainBearing.mass * (self.mainBearing.cm[i] - self.cm[i]) ** 2) + \
                                  (self.secondBearing.mass * (self.secondBearing.cm[i] - self.cm[i]) ** 2)
        self.I = I

#-------------------------------------------------------------------------------

class Gearbox():
    implements(SubComponent)  
    ''' Gearbox class          
          The Gearbox class is used to represent the gearbox component of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    
    def __init__(self,iDsgn,RotorTorque,GearRatio,GearConfig,Bevel,RotorDiam):
        '''
        Initializes gearbox component 
          
        Parameters
        ----------
        iDsgn : int
          Integer which selects the type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive
          Method is currently configured only for type 1 drivetrains though the configuration can be single, double or triple staged with any combination of epicyclic and parallel stages.
        RotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        MachineRating : float
          The power rating for the overall wind turbine [kW]
        GearRatio : float
          Ratio of high speed to low speed shaft based on total gear ratio of gearbox
        GearConfig : str
          String that represents the configuration of the gearbox (stage number and types).
          Possible configurations include 'e', 'p', 'pp', 'ep', 'ee', 'epp', 'eep', 'eee'.
        Bevel : int
          Flag for the presence of a bevel stage - 1 if present, 0 if not; typically it is not present.
        RotorDiam : float
          The wind turbine rotor diameter [m]
        stagemass : array
          Array of stage masses for the gearbox [kg]
        length : float
          Gearbox critical length [m]
        height : float
          Gearbox critical height [m]
        diameter : float
          Gearbox diameter [m]
        '''

        self.stagemass = [None, 0.0, 0.0, 0.0, 0.0]

        # compute masses, dimensions and cost
        overallweightFact = 1.00                          # default weight factor 1.0 (should depend on drivetrain design)
        self.stagemass = [None, 0.0, 0.0, 0.0, 0.0]       # TODO: problem initializing stagemass and accessing in compute
 
        # find weight of each stage depending on configuration
        # Gear ratio reduced for each stage based on principle that mixed epicyclic/parallel trains have a final stage ratio set at 1:2.5
        if GearConfig == 'p':
            self.stagemass[1] = self.__getParallelStageWeight(RotorTorque,GearRatio)
        if GearConfig == 'e':
            self.stagemass[1] = self.__getEpicyclicStageWeight(RotorTorque,GearRatio)
        if GearConfig == 'pp':
            self.stagemass[1] = self.__getParallelStageWeight(RotorTorque,GearRatio**0.5)
            self.stagemass[2] = self.__getParallelStageWeight(RotorTorque,GearRatio**0.5)
        if GearConfig == 'ep':
            self.stagemass[1] = self.__getEpicyclicStageWeight(RotorTorque,GearRatio/2.5)
            self.stagemass[2] = self.__getParallelStageWeight(RotorTorque,2.5)
        if GearConfig == 'ee':
            self.stagemass[1] = self.__getEpicyclicStageWeight(RotorTorque,GearRatio**0.5)
            self.stagemass[2] = self.__getEpicyclicStageWeight(RotorTorque,GearRatio**0.5)
        if GearConfig == 'eep':
            U1 = (GearRatio/3.0)**0.5
            U2 = (GearRatio/3.0)**0.5
            U3 = 3.0
            self.stagemass[1] = self.__getEpicyclicStageWeight(RotorTorque,1,U1,U2,U3)  #different than sunderland
            self.stagemass[2] = self.__getEpicyclicStageWeight(RotorTorque,2,U1,U2,U3)
            self.stagemass[3] = self.__getParallelStageWeight(RotorTorque,3,U1,U2,U3)
        if GearConfig == 'epp':
            U1 = GearRatio**0.33*1.4
            U2 = (GearRatio**0.33/1.18)
            U3 = (GearRatio**0.33/1.18)
            self.stagemass[1] = self.__getEpicyclicStageWeight(RotorTorque,1,U1,U2,U3)    # not in sunderland
            self.stagemass[2] = self.__getParallelStageWeight(RotorTorque,2,U1,U2,U3)
            self.stagemass[3] = self.__getParallelStageWeight(RotorTorque,3,U1,U2,U3)
        if GearConfig == 'eee':
            self.stagemass[1] = self.__getEpicyclicStageWeight(RotorTorque,GearRatio**(0.33))
            self.stagemass[2] = self.__getEpicyclicStageWeight(RotorTorque,GearRatio**(0.33))
            self.stagemass[3] = self.__getEpicyclicStageWeight(RotorTorque,GearRatio**(0.33))
        if GearConfig == 'ppp':
            self.stagemass[1] = self.__getParallelStageWeight(RotorTorque,GearRatio**(0.33))
            self.stagemass[2] = self.__getParallelStageWeight(RotorTorque,GearRatio**(0.33))
            self.stagemass[3] = self.__getParallelStageWeight(RotorTorque,GearRatio**(0.33))


        if (Bevel):
            self.stagemass[4] = 0.0454 * (RotorTorque ** 0.85)

        mass = 0.0
        for i in range(1,4):
            mass += self.stagemass[i]
        mass     *= overallweightFact  
        self.mass = (mass)

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]   = cm[1] = 0.0
        cm[2]   = 0.025 * RotorDiam
        self.cm = cm

        self.length = (0.012 * RotorDiam)
        self.height = (0.015 * RotorDiam)
        self.diameter = (0.75 * self.height)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = self.mass * (self.diameter ** 2 ) / 8 + (self.mass / 2) * (self.height ** 2) / 8
        I[1] = self.mass * (0.5 * (self.diameter ** 2) + (2 / 3) * (self.length ** 2) + 0.25 * (self.height ** 2)) / 8
        I[2] = I[1]       
        self.I = I

    def __getParallelStageWeight(self,RotorTorque,stage,StageRatio1,StageRatio2,StageRatio3):

        ''' 
          This method calculates the stage weight for a parallel stage in a gearbox based on the input torque, stage number, and stage ratio for each individual stage.
        '''
        
        serviceFact     = 1.00                                # default service factor for a gear stage is 1.75 based on full span VP (should depend on control type)
        applicationFact = 0.4                             # application factor ???        
        stageweightFact = 8.029 #/2                           # stage weight factor applied to each Gearbox stage

        if (RotorTorque * serviceFact) < 200000.0:       # design factor for design and manufacture of Gearbox
            designFact = 925.0
        elif (RotorTorque * serviceFact) < 700000.0:
            designFact = 1000.0
        else:
            designFact = 1100.0                            # TODO: should be an exception for all 2 stage Gearboxes to have designFact = 1000
            
        if stage == 1:
            Qr         = RotorTorque
            StageRatio = StageRatio1
        elif stage == 2:
            Qr         = RotorTorque/StageRatio1
            StageRatio = StageRatio2
        elif stage == 3:
            Qr         = RotorTorque/(StageRatio1*StageRatio2)
            StageRatio = StageRatio3

        gearFact = applicationFact / designFact          # Gearbox factor for design, manufacture and application of Gearbox
        
        gearweightFact = 1 + (1 / StageRatio) + StageRatio + (StageRatio ** 2)
                                                         # Gearbox weight factor for relationship of stage ratio required and relative stage volume

        stageWeight = stageweightFact * Qr * serviceFact * gearFact * gearweightFact
                                                         # forumula for parallel gearstage weight based on sunderland model

        return stageWeight

    def __getEpicyclicStageWeight(self,RotorTorque,stage,StageRatio1,StageRatio2,StageRatio3):
        ''' 
          This method calculates the stage weight for a epicyclic stage in a gearbox based on the input torque, stage number, and stage ratio for each individual stage
        '''

        serviceFact     = 1.00                                # default service factor for a gear stage is 1.75 based on full span VP (should depend on control type)
        applicationFact = 0.4                             # application factor ???        
        stageweightFact = 8.029/12                          # stage weight factor applied to each Gearbox stage
        OptWheels       = 3.0                                    # default optional wheels (should depend on stage design)

        if (RotorTorque * serviceFact) < 200000.0:       # design factor for design and manufacture of Gearbox
            designFact = 850.0
        elif (RotorTorque * serviceFact) < 700000.0:
            designFact = 950.0
        else:
            designFact = 1100.0
           
        if stage == 1:
            Qr         = RotorTorque
            StageRatio = StageRatio1
        elif stage == 2:
            Qr         = RotorTorque/StageRatio1
            StageRatio = StageRatio2
        elif stage == 3:
            Qr         = RotorTorque/(StageRatio1*StageRatio2)
            StageRatio = StageRatio3

        gearFact = applicationFact / designFact          # Gearbox factor for design, manufacture and application of Gearbox
       
        sunwheelratio  = (StageRatio / 2.0) - 1             # sun wheel ratio for epicyclic Gearbox stage based on stage ratio
        gearweightFact = (1 / OptWheels) + (1 / (OptWheels * sunwheelratio)) + sunwheelratio + \
                         ((1 + sunwheelratio) / OptWheels) * ((StageRatio - 1.) ** 2)
                                                         # Gearbox weight factor for relationship of stage ratio required and relative stage volume

        stageWeight    = stageweightFact * Qr * serviceFact * gearFact * gearweightFact
                                                         # forumula for epicyclic gearstage weight based on sunderland model

        return stageWeight

    def getStageMass(self):
        '''
        This method returns an array of the stage masses for individual stages in a gearbox
       
        Returns
        -------
        self.stagemass : array
           Array of individual stage masses for a gearbox
        '''

        return self.stagemass        

#-------------------------------------------------------------------------------

class HighSpeedShaft():
	  implements(SubComponent)
	  ''' 
	     Basic class for a high speed shaft.
	  '''

class MechanicalBrake():
	  implements(SubComponent)
	  '''
	     Basic class for mechanical brake.
	  '''
              
class HighSpeedSide():
    implements(SubComponent)
    ''' 
    HighSpeedShaft class          
          The HighSpeedShaft class is used to represent the high speed shaft and mechanical brake components of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''

    def __init__(self,MachineRating,RotorTorque,GearRatio,RotorDiam,lssOutDiam):
        ''' Initializes high speed shaft and mechanical brake component 
          
        Parameters
        ----------
        MachineRating : float
          The power rating for the overall wind turbine [kW]
        RotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        GearRatio : float
          Ratio of high speed to low speed shaft based on total gear ratio of gearbox
        RotorDiam : float
          The wind turbine rotor diameter [m]
        lssOutDiam : float
          outer diameter of low speed shaft [m]    
        diameter : float
          diameter of high speed shaft
        length : float
          length of high speed shaft    
        '''

        self.HSS = HighSpeedShaft()
        self.MechBrake = MechanicalBrake()
        
        # compute masses, dimensions and cost        
        designTQ = RotorTorque / GearRatio               # design torque [Nm] based on rotor torque and Gearbox ratio
        massFact = 0.025                                 # mass matching factor default value
        self.HSS.mass = (massFact * designTQ)
        
        self.MechBrake.mass = (0.5 * self.HSS.mass)      # relationship derived from HSS multiplier for University of Sunderland model compared to NREL CSM for 750 kW and 1.5 MW turbines

        self.mass = (self.MechBrake.mass + self.HSS.mass)                      

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]   = 0.5 * (0.0125 * RotorDiam)
        cm[1]   = 0.0
        cm[2]   = 0.025 * RotorDiam
        self.cm = cm

        self.diameter = (1.5 * lssOutDiam)                     # based on WindPACT relationships for full HSS / mechanical brake assembly
        self.length = (0.025)

        I = np.array([0.0, 0.0, 0.0])
        I[0]    = 0.25 * self.length * 3.14159 * (self.diameter ** 2) * GearRatio * (self.diameter ** 2) / 8
        I[1]    = self.mass * ((3/4) * (self.diameter ** 2) + (self.length ** 2)) / 12
        I[2]    = I[1]      
        self.I = I

#-------------------------------------------------------------------------------

class Generator():
    implements(SubComponent)
    '''Generator class          
          The Generator class is used to represent the generator of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    
    def __init__(self,iDsgn,MachineRating,RotorSpeed,RotorDiam,GearRatio):
        ''' Initializes generator component 
        
        Parameters
        ----------
        iDsgn : int
          Integer which selects the type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive
          Method is currently configured only for type 1 drivetrains.
        MachineRating : float
          The power rating for the overall wind turbine [kW]
        RotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        GearRatio : float
          Ratio of high speed to low speed shaft based on total gear ratio of gearbox
        RotorDiam : float
          The wind turbine rotor diameter [m]
        length : float
          Generator critical length [m]
        width : float
          Generator critical width [m]
        depth : float
          Generator critical depth [m]
        '''

        massCoeff = [None, 6.4737, 10.51 ,  5.34  , 37.68  ]           
        massExp   = [None, 0.9223, 0.9223,  0.9223, 1      ]

        CalcRPM    = 80 / (RotorDiam*0.5*pi/30)
        CalcTorque = (MachineRating*1.1) / (CalcRPM * pi/30)
        
        if (iDsgn < 4):
            self.mass = (massCoeff[iDsgn] * MachineRating ** massExp[iDsgn])   
        else:  # direct drive
            self.mass = (massCoeff[iDsgn] * CalcTorque ** massExp[iDsgn])  

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]  = 0.0125 * RotorDiam
        cm[1]  = 0.0
        cm[2]  = 0.025 * RotorDiam
        self.cm = cm

        self.length = (1.6 * 0.015 * RotorDiam)
        self.depth = (0.015 * RotorDiam)
        self.width = (0.5 * self.depth)

        I = np.array([0.0, 0.0, 0.0])
        I[0]   = ((4.86 * (10 ** (-5))) * (RotorDiam ** 5.333)) + (((2/3) * self.mass) * (self.depth ** 2 + self.width ** 2) / 8)
        I[1]   = (I[0] / 2) / (GearRatio ** 2) + ((1/3) * self.mass * (self.length ** 2) / 12) + (((2 / 3) * self.mass) * \
                   (self.depth ** 2 + self.width ** 2 + (4/3) * (self.length ** 2)) / 16 )
        I[2]   = I[1]                           
        self.I = I

#-------------------------------------------------------------------------------

class Bedplate():
    implements(SubComponent)
    ''' Bedplate class         
          The Bedplate class is used to represent the bedplate of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    
    def __init__(self,iDsgn,RotorTorque,RotorMass,RotorThrust,RotorDiam,TowerTopDiam):
        ''' Initializes bedplate component 
        
        Parameters
        ----------
        iDsgn : int
          Integer which selects the type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive
          Method is currently configured only for type 1 drivetrains.
        RotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        RotorMass : float
          Mass of the rotor [kg]
        RotorThrust : float
          Maximum thrust from the rotor applied to the drivetrain under extreme conditions [N]
        RotorDiam : float
          The wind turbine rotor diameter [m]
        TowerTopDiam : float
          Diameter of the turbine tower top [m] 
        length : float
          Bedplate critical length [m]
        width : float
          Bedplate critical width [m]
        area : float
          Bedplate base area [m^2]
        height : float
          Bedplate critical height [m]
        depth : float
          Bedplate critical depth [m]
        '''

        # compute masses, dimensions and cost
        # bedplate sizing based on superposition of loads for rotor torque, thurst, weight         #TODO: only handles bedplate for a traditional drivetrain configuration
        bedplateWeightFact = 2.86                                   # toruqe weight factor for bedplate (should depend on drivetrain, bedplate type)

        torqueweightCoeff = 0.00368                   # regression coefficient multiplier for bedplate weight based on rotor torque
        MassFromTorque    = bedplateWeightFact * (torqueweightCoeff * RotorTorque)
                                                                  
        thrustweightCoeff = 0.00158                                 # regression coefficient multiplier for bedplate weight based on rotor thrust
        MassFromThrust    = bedplateWeightFact * (thrustweightCoeff * (RotorThrust * TowerTopDiam))

        rotorweightCoeff    = 0.015                                    # regression coefficient multiplier for bedplate weight based on rotor weight
        MassFromRotorWeight = bedplateWeightFact * (rotorweightCoeff * (RotorMass * TowerTopDiam))
        
        # additional weight ascribed to bedplate area
        BPlengthFact    = 1.5874                                       # bedplate length factor (should depend on drivetrain, bedplate type)
        nacellevolFact  = 0.052                                      # nacelle volume factor (should depend on drivetrain, bedplate type)
        self.length = (BPlengthFact * nacellevolFact * RotorDiam)     # bedplate length [m] calculated as a function of rotor diameter
        self.width = (self.length / 2.0)                              # bedplate width [m] assumed to be half of bedplate length
        self.area       = self.length * self.width                        # bedplate area [m^2]
        self.height = ((2 / 3) * self.length)                         # bedplate height [m] calculated based on cladding area
        areaweightCoeff = 100                                       # regression coefficient multiplier for bedplate weight based on bedplate area
        MassFromArea    = bedplateWeightFact * (areaweightCoeff * self.area)
    
        # total mass is calculated based on adding masses attributed to rotor torque, thrust, weight and bedplate area
        TotalMass = MassFromTorque + MassFromThrust + MassFromRotorWeight + MassFromArea

        # for single-stage and multi-generator - bedplate mass based on regresstion to rotor diameter
        # for geared and direct drive - bedplate mass based on total mass as calculated above
        massCoeff    = [None,22448,1.29490,1.72080,22448 ]
        massExp      = [None,    0,1.9525, 1.9525 ,    0 ]
        massCoeff[1] = TotalMass  
        ddweightfact = 0.55                                         # direct drive bedplate weight assumed to be 55% of modular geared type
        massCoeff[4] = TotalMass * ddweightfact

        self.mass = (massCoeff[iDsgn] * RotorDiam ** massExp[iDsgn] )
        
        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = cm[1] = 0.0
        cm[2] = 0.0122 * RotorDiam                             # half distance from shaft to yaw axis
        self.cm = cm

        self.depth = (self.length / 2.0)

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.width ** 2 + self.depth ** 2) / 8
        I[1]  = self.mass * (self.depth ** 2 + self.width ** 2 + (4/3) * self.length ** 2) / 16
        I[2]  = I[1]                          
        self.I = I

#-------------------------------------------------------------------------------
   
class YawSystem():
    implements(SubComponent)
    ''' YawSystem class
          The YawSystem class is used to represent the yaw system of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
 
    def __init__(self,RotorDiam,RotorThrust,TowerTopDiam,AboveYawMass):
        ''' Initializes yaw system 
        
        Parameters
        ----------
        RotorDiam : float
          The wind turbine rotor diameter [m]
        RotorThrust : float
          Maximum thrust from the rotor applied to the drivetrain under extreme conditions [N]
        TowerTopDiam : float
          Diameter of the turbine tower top [m]
        AboveYawMass : float
          Mass of the system above the yaw bearing [kg]
        '''
        
        # yaw weight depends on moment due to weight of components above yaw bearing and moment due to max thrust load
        #AboveYawMass = 350000 # verboseging number based on 5 MW RNA mass
        yawfactor = 0.41 * (2.4 * (10 ** (-3)))                   # should depend on rotor configuration: blade number and hub type
        weightMom = AboveYawMass * RotorDiam                    # moment due to weight above yaw system
        thrustMom = RotorThrust * TowerTopDiam                  # moment due to rotor thrust
        self.mass = (yawfactor * (0.4 * weightMom + 0.975 * thrustMom))
        
        # calculate mass properties
        # yaw system assumed to be collocated to tower top center              
        cm = np.array([0.0,0.0,0.0])
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        self.I = I

if __name__ == '__main__':
     pass