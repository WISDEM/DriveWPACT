from math import *
from common import SubComponent
import numpy as np
from zope.interface import implements

from nacelle_components import LowSpeedShaft, MainBearings, Gearbox, HighSpeedSide, Generator, Bedplate, YawSystem

#-------------------------------------------------------------------------------

class NacelleSystem(): # changed name to nacelle - need to rename, move code pieces, develop configurations ***
    implements(SubComponent)
    ''' NacelleSystem class       
          The Nacelle class is used to represent the overall nacelle of a wind turbine.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def __init__(self,RotorSpeed, RotorTorque, RotorThrust, RotorMass, RotorDiam, iDsgn, MachineRating, GearRatio, GearConfig, Bevel, TowerTopDiam, crane):
        ''' 
        Initializes nacelle system 
        
        Parameters
        ----------
        RotorSpeed : float
          Speed of the rotor at rated power [rpm]
        RotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        RotorThrust : float
          Maximum thrust from the rotor applied to the drivetrain under extreme conditions [N]
        RotorMass : float
          The wind turbine rotor mass [kg]
        RotorDiam : float
          The wind turbine rotor diameter [m]
        iDsgn : int
          Integer which selects the type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive
          Method is currently configured only for type 1 drivetrains though the configuration can be single, double or triple staged with any combination of epicyclic and parallel stages.
        MachineRating : float
          The power rating for the overall wind turbine [kW]
        GearRatio : float
          Ratio of high speed to low speed shaft based on total gear ratio of gearbox
        GearConfig : str
          String that represents the configuration of the gearbox (stage number and types).
          Possible configurations include 'e', 'p', 'pp', 'ep', 'ee', 'epp', 'eep', 'eee'.
        Bevel : int
          Flag for the presence of a bevel stage - 1 if present, 0 if not; typically it is not present.
        TowerTopDiam : float
          Diameter of the turbine tower top [m] 
        crane   : bool
          flag for presence of service crane up-tower
        lss  : LowSpeedShaft()
        mbg  : MainBearings()
        gear : Gearbox()
        hss  : HighSpeedSide()
        gen  : Generator()
        bpl  : Bedplate()
        yaw  : YawSystem()            
        econnectionsCost : float     
          cost for electrical cabling
        econnectionsMass : float    
          mass uptower for electrical cabling [kg]
        vspdEtronicsCost : float    
          cost for variable speed electronics
        vspdEtronicsMass : float    
          mass for variable speed electronics [kg]
        hydrCoolingCost : float     
          cost for hydraulics and HVAC system
        hydrCoolingMass : float     
          mass for hydraulics and HVAC system [kg]
        ControlsCost : float  
          cost for controls up      
        ControlsMass : float  
          mass uptower for controls [kg]      
        nacellePlatformsCost : float
          cost for nacelle platforms
        nacellePlatformsMass : float
          mass for nacelle platforms [kg]
        craneCost : float     
          cost for service crane uptower       
        craneMass : float           
          mass for service crane uptower [kg]
        mainframeCost : float       
          cost for mainframe including bedplate, service crane and platforms
        mainframeMass : float       
          mass for mainframe including bedplate, service crane and platforms [kg]
        nacelleCovCost : float      
          cost for nacelle cover
        nacelleCovMass : float  
          mass for nacelle cover [kg] 
        '''

        # create instances of input variables and initialize values
        self.lss = LowSpeedShaft(RotorDiam,RotorMass,RotorTorque)
        self.mbg = MainBearings(self.lss, RotorSpeed, RotorDiam)
        self.gear = Gearbox(iDsgn,RotorTorque,GearRatio,GearConfig,Bevel,RotorDiam)
        self.stagemass = self.gear.getStageMass() #return gearbox stage masses
        self.hss = HighSpeedSide(MachineRating,RotorTorque,GearRatio,RotorDiam,self.lss.diameter)
        self.gen = Generator(iDsgn,MachineRating,RotorSpeed,RotorDiam,GearRatio)
        self.bpl = Bedplate(iDsgn,RotorTorque,RotorMass,RotorThrust,RotorDiam,TowerTopDiam) 
        self.aboveYawMass = 0.0


        # initialize default status for onboard crane and onshore/offshroe
        self.crane   = crane

        # initialize other drivetrain components
        self.econnectionsCost     = 0.0
        self.econnectionsMass     = 0.0
        self.vspdEtronicsCost     = 0.0
        self.vspdEtronicsMass     = 0.0
        self.hydrCoolingCost      = 0.0
        self.hydrCoolingMass      = 0.0
        self.ControlsCost         = 0.0
        self.ControlsMass         = 0.0
        self.nacellePlatformsCost = 0.0
        self.nacellePlatformsMass = 0.0
        self.craneCost            = 0.0
        self.craneMass            = 0.0
        self.mainframeCost        = 0.0
        self.mainframeMass        = 0.0
        self.nacelleCovCost       = 0.0
        self.nacelleCovMass       = 0.0

        self.crane = crane
        self.GearConfig = GearConfig
        
        #computation of mass for main drivetrian subsystems     
        self.stagemass = self.gear.getStageMass() #return gearbox stage masses

        # electronic systems, hydraulics and controls 
        self.econnectionsMass = 0.0

        self.vspdEtronicsMass = 0.0
               
        self.hydrCoolingMass = 0.08 * MachineRating
 
        self.ControlsMass     = 0.0

        # mainframe system including bedplate, platforms, crane and miscellaneous hardware
        self.nacellePlatformsMass = 0.125 * self.bpl.mass

        if (self.crane):
            self.craneMass =  3000.0
        else:
            self.craneMass = 0.0
 
        self.mainframeMass  = self.bpl.mass + self.craneMass + self.nacellePlatformsMass     
        
        nacelleCovArea      = 2 * (self.bpl.length ** 2)              # this calculation is based on Sunderland
        self.nacelleCovMass = (84.1 * nacelleCovArea) / 2          # this calculation is based on Sunderland - divided by 2 in order to approach CSM

        self.length      = self.bpl.length                              # nacelle length [m] based on bedplate length
        self.width       = self.bpl.width                        # nacelle width [m] based on bedplate width
        self.height      = (2.0 / 3.0) * self.length                         # nacelle height [m] calculated based on cladding area
        
        # yaw system weight calculations based on total system mass above yaw system
        self.aboveYawMass =  self.lss.mass + \
                    self.mbg.mass + \
                    self.gear.mass + \
                    self.hss.mass + \
                    self.gen.mass + \
                    self.mainframeMass + \
                    self.econnectionsMass + \
                    self.vspdEtronicsMass + \
                    self.hydrCoolingMass + \
                    self.nacelleCovMass
        self.yaw = YawSystem(RotorDiam,RotorThrust,TowerTopDiam,self.aboveYawMass)   # yaw mass calculation based on Sunderalnd model

        # aggregation of nacelle mass
        self.mass = (self.aboveYawMass + self.yaw.mass)

        # calculation of mass center and moments of inertia
        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass
            cm[i] = (self.lss.mass * self.lss.cm[i] + 
                    self.mbg.mainBearing.mass * self.mbg.mainBearing.cm[i] + self.mbg.secondBearing.mass * self.mbg.secondBearing.cm[i] + \
                    self.gear.mass * self.gear.cm[i] + self.hss.mass * self.hss.cm[i] + \
                    self.gen.mass * self.gen.cm[i] + self.bpl.mass * self.bpl.cm[i] ) / \
                    (self.lss.mass + self.mbg.mainBearing.mass + self.mbg.secondBearing.mass + \
                    self.gear.mass + self.hss.mass + self.gen.mass + self.bpl.mass)
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM
            I[i]  =  self.lss.I[i] + self.mbg.mainBearing.I[i] + self.mbg.secondBearing.I[i] + self.gear.I[i] + \
                          self.hss.I[i] + self.gen.I[i] + self.bpl.I[i]
            # translate to nacelle CM using parallel axis theorem
            for j in (range(0,3)): 
                if i != j:
                    I[i] +=  self.lss.mass * (self.lss.cm[i] - cm[i]) ** 2 + \
                                  self.mbg.mainBearing.mass * (self.mbg.mainBearing.cm[i] - cm[i]) ** 2 + \
                                  self.mbg.secondBearing.mass * (self.mbg.secondBearing.cm[i] - cm[i]) ** 2 + \
                                  self.gear.mass * (self.gear.cm[i] - cm[i]) ** 2 + \
                                  self.hss.mass * (self.hss.cm[i] - cm[i]) ** 2 + \
                                  self.gen.mass * (self.gen.cm[i] - cm[i]) ** 2 + \
                                  self.bpl.mass * (self.bpl.cm[i] - cm[i]) ** 2
        self.I = I

    def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses
        
        detailedMasses : array_like of float
           detailed masses for nacelle components
        """
        
        detailedMasses = [self.lss.mass, self.mbg.mass, self.gear.mass, self.hss.mass, self.gen.mass, self.vspdEtronicsMass, \
                self.econnectionsMass, self.hydrCoolingMass, \
                self.ControlsMass, self.yaw.mass, self.mainframeMass, self.nacelleCovMass]

        return detailedMasses

#------------------------------------------------------------------

def example():

    # test of module for turbine data set

    # NREL 5 MW Rotor Variables
    RotorDiam = 126.0 # m
    RotorSpeed = 11.05 # m/s
    RotorTorque = 4365248.74 # Nm
    RotorThrust = 500930.84 # N
    RotorMass = 142585.75 # kg

    # NREL 5 MW Drivetrain variables
    iDsgn = 1 # geared 3-stage Gearbox with induction generator machine
    MachineRating = 5000.0 # kW
    GearRatio = 97.0 # 97:1 as listed in the 5 MW reference document
    GearConfig = 'eep' # epicyclic-epicyclic-parallel
    Bevel = 0 # no bevel stage
    crane = True # onboard crane present

    # NREL 5 MW Tower Variables
    TowerTopDiam = 3.78 # m

    print '----- NREL 5 MW Turbine -----'
    nace = NacelleSystem(RotorSpeed, RotorTorque, RotorThrust, RotorMass, RotorDiam, iDsgn, \
                   MachineRating, GearRatio, GearConfig, Bevel, TowerTopDiam, crane)

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg  %6.2f m length %6.2f m OD %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lss.mass , nace.lss.length,nace.lss.diameter, nace.lss.I[0], nace.lss.I[1], nace.lss.I[2], nace.lss.cm[0], nace.lss.cm[1], nace.lss.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' % (nace.mbg.mass , nace.mbg.I[0], nace.mbg.I[1], nace.mbg.I[2], nace.mbg.cm[0], nace.mbg.cm[1], nace.mbg.cm[2])
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f m length %6.2f m diameter %6.2f m height %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gear.mass, nace.gear.length, nace.gear.diameter, nace.gear.height, nace.gear.I[0], nace.gear.I[1], nace.gear.I[2], nace.gear.cm[0], nace.gear.cm[1], nace.gear.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.stagemass[1], nace.stagemass[2], nace.stagemass[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f m length %6.2f m OD %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.hss.mass, nace.hss.length, nace.hss.diameter, nace.hss.I[0], nace.hss.I[1], nace.hss.I[2], nace.hss.cm[0], nace.hss.cm[1], nace.hss.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f m length %6.2f m width %6.2f m depth %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gen.mass, nace.gen.length, nace.gen.width, nace.gen.depth, nace.gen.I[0], nace.gen.I[1], nace.gen.I[2], nace.gen.cm[0], nace.gen.cm[1], nace.gen.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg %6.2f m length %6.2f m width' % (nace.mainframeMass, nace.bpl.length, nace.bpl.width)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bpl.mass, nace.bpl.I[0], nace.bpl.I[1], nace.bpl.I[2], nace.bpl.cm[0], nace.bpl.cm[1], nace.bpl.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.nacelleCovMass , nace.height, nace.width, nace.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yaw.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1


def example2():

    # WindPACT 1.5 MW Drivetrain variables
    iDsgn = 1 # geared 3-stage Gearbox with induction generator machine
    MachineRating = 1500 # machine rating [kW]
    GearRatio = 87.965
    GearConfig = 'epp'
    Bevel = 0
    crane = True

    # WindPACT 1.5 MW Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 80 # max tip speed [m/s]
    RotorDiam = 70 # rotor diameter [m]
    RotorSpeed = 21.830
    DrivetrainEfficiency = 0.95
    RotorTorque = (MachineRating * 1000 / DrivetrainEfficiency) / (RotorSpeed * (pi / 30)) 
        # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    RotorThrust = 324000 
    RotorMass = 28560 # rotor mass [kg]

    # WindPACT 1.5 MW Tower Variables
    TowerTopDiam = 2.7 # tower top diameter [m]

    print '----- WindPACT 1.5 MW Turbine -----'
    nace = NacelleSystem(RotorSpeed, RotorTorque, RotorThrust, RotorMass, RotorDiam, iDsgn, \
                   MachineRating, GearRatio, GearConfig, Bevel, TowerTopDiam, crane)

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg  %6.2f m length %6.2f m OD %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lss.mass , nace.lss.length,nace.lss.diameter, nace.lss.I[0], nace.lss.I[1], nace.lss.I[2], nace.lss.cm[0], nace.lss.cm[1], nace.lss.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' % (nace.mbg.mass , nace.mbg.I[0], nace.mbg.I[1], nace.mbg.I[2], nace.mbg.cm[0], nace.mbg.cm[1], nace.mbg.cm[2])
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f m length %6.2f m diameter %6.2f m height %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gear.mass, nace.gear.length, nace.gear.diameter, nace.gear.height, nace.gear.I[0], nace.gear.I[1], nace.gear.I[2], nace.gear.cm[0], nace.gear.cm[1], nace.gear.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.stagemass[1], nace.stagemass[2], nace.stagemass[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f m length %6.2f m OD %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.hss.mass, nace.hss.length, nace.hss.diameter, nace.hss.I[0], nace.hss.I[1], nace.hss.I[2], nace.hss.cm[0], nace.hss.cm[1], nace.hss.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f m length %6.2f m width %6.2f m depth %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gen.mass, nace.gen.length, nace.gen.width, nace.gen.depth, nace.gen.I[0], nace.gen.I[1], nace.gen.I[2], nace.gen.cm[0], nace.gen.cm[1], nace.gen.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg %6.2f m length %6.2f m width' % (nace.mainframeMass, nace.bpl.length, nace.bpl.width)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bpl.mass, nace.bpl.I[0], nace.bpl.I[1], nace.bpl.I[2], nace.bpl.cm[0], nace.bpl.cm[1], nace.bpl.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.nacelleCovMass , nace.height, nace.width, nace.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yaw.mass)
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1
 
    # GRC Drivetrain variables
    iDesign = 1 # geared 3-stage Gearbox with induction generator machine
    MachineRating = 750 # machine rating [kW]
    GearRatio = 81.491 
    GearConfig = 'epp'
    Bevel = 0

    # GRC Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 8 # max tip speed [m/s]
    RotorDiam = 48.2 # rotor diameter [m]
    #RotorSpeed = MaxTipSpeed / ((RotorDiam / 2) * (pi / 30)) # max / rated rotor speed [rpm] calculated from max tip speed and rotor diamter
    RotorSpeed = 22
    DrivetrainEfficiency = 0.944
    RotorTorque = (MachineRating * 1000 / DrivetrainEfficiency) / (RotorSpeed * (pi / 30)) # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    RotorThrust = 159000 # based on windpact 750 kW design (GRC information not available)
    RotorMass = 13200 # rotor mass [kg]

    # Tower Variables
    TowerTopDiam = 2 # tower top diameter [m] - not given

    print '----- GRC 750 kW Turbine -----'
    nace = NacelleSystem(RotorSpeed, RotorTorque, RotorThrust, RotorMass, RotorDiam, iDsgn, \
                   MachineRating, GearRatio, GearConfig, Bevel, TowerTopDiam, crane)

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg  %6.2f m length %6.2f m OD %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lss.mass , nace.lss.length,nace.lss.diameter, nace.lss.I[0], nace.lss.I[1], nace.lss.I[2], nace.lss.cm[0], nace.lss.cm[1], nace.lss.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' % (nace.mbg.mass , nace.mbg.I[0], nace.mbg.I[1], nace.mbg.I[2], nace.mbg.cm[0], nace.mbg.cm[1], nace.mbg.cm[2])
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f m length %6.2f m diameter %6.2f m height %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gear.mass, nace.gear.length, nace.gear.diameter, nace.gear.height, nace.gear.I[0], nace.gear.I[1], nace.gear.I[2], nace.gear.cm[0], nace.gear.cm[1], nace.gear.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.stagemass[1], nace.stagemass[2], nace.stagemass[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f m length %6.2f m OD %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.hss.mass, nace.hss.length, nace.hss.diameter, nace.hss.I[0], nace.hss.I[1], nace.hss.I[2], nace.hss.cm[0], nace.hss.cm[1], nace.hss.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f m length %6.2f m width %6.2f m depth %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gen.mass, nace.gen.length, nace.gen.width, nace.gen.depth, nace.gen.I[0], nace.gen.I[1], nace.gen.I[2], nace.gen.cm[0], nace.gen.cm[1], nace.gen.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg %6.2f m length %6.2f m width' % (nace.mainframeMass, nace.bpl.length, nace.bpl.width)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bpl.mass, nace.bpl.I[0], nace.bpl.I[1], nace.bpl.I[2], nace.bpl.cm[0], nace.bpl.cm[1], nace.bpl.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.nacelleCovMass , nace.height, nace.width, nace.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yaw.mass)
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1

    # Alstom Drivetrain variables
    iDesign = 1 # geared 3-stage Gearbox with induction generator machine
    MachineRating = 3000 # machine rating [kW]
    GearRatio = 102.19 # 
    GearConfig = 'eep'
    Bevel = 0

    # Alstom Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 80 # max tip speed [m/s]
    RotorDiam = 100.8 # rotor diameter [m]
    #RotorSpeed = MaxTipSpeed / ((RotorDiam / 2) * (pi / 30)) # max / rated rotor speed [rpm] calculated from max tip speed and rotor diamter
    RotorSpeed = 15.1 # based on windpact 3 MW
    DrivetrainEfficiency = 0.95
    RotorTorque = (MachineRating * 1000 / DrivetrainEfficiency) / (RotorSpeed * (pi / 30)) # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    RotorThrust = 797000 # based on windpact 3.0 MW - Alstom thrust not provided
    RotorMass = 49498 # rotor mass [kg] - not given - using Windpact 3.0 MW

    # Tower Variables
    TowerTopDiam = 3.5 # tower top diameter [m] - not given

    print '----- Alstom 3.0 MW Turbine -----'
    nace = NacelleSystem(RotorSpeed, RotorTorque, RotorThrust, RotorMass, RotorDiam, iDsgn, \
                   MachineRating, GearRatio, GearConfig, Bevel, TowerTopDiam, crane)

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg  %6.2f m length %6.2f m OD %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lss.mass , nace.lss.length,nace.lss.diameter, nace.lss.I[0], nace.lss.I[1], nace.lss.I[2], nace.lss.cm[0], nace.lss.cm[1], nace.lss.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' % (nace.mbg.mass , nace.mbg.I[0], nace.mbg.I[1], nace.mbg.I[2], nace.mbg.cm[0], nace.mbg.cm[1], nace.mbg.cm[2])
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f m length %6.2f m diameter %6.2f m height %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gear.mass, nace.gear.length, nace.gear.diameter, nace.gear.height, nace.gear.I[0], nace.gear.I[1], nace.gear.I[2], nace.gear.cm[0], nace.gear.cm[1], nace.gear.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.stagemass[1], nace.stagemass[2], nace.stagemass[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f m length %6.2f m OD %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.hss.mass, nace.hss.length, nace.hss.diameter, nace.hss.I[0], nace.hss.I[1], nace.hss.I[2], nace.hss.cm[0], nace.hss.cm[1], nace.hss.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f m length %6.2f m width %6.2f m depth %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gen.mass, nace.gen.length, nace.gen.width, nace.gen.depth, nace.gen.I[0], nace.gen.I[1], nace.gen.I[2], nace.gen.cm[0], nace.gen.cm[1], nace.gen.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg %6.2f m length %6.2f m width' % (nace.mainframeMass, nace.bpl.length, nace.bpl.width)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bpl.mass, nace.bpl.I[0], nace.bpl.I[1], nace.bpl.I[2], nace.bpl.cm[0], nace.bpl.cm[1], nace.bpl.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.nacelleCovMass , nace.height, nace.width, nace.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yaw.mass)
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1


if __name__ == '__main__':
    ''' Main runs through tests of several drivetrain configurations with known component masses and dimensions '''
    
    # todo: adjust to use rotor model interface

    example()
    
    example2()