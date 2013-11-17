"""
driveSE_components.py
New components for hub, low speed shaft, main bearings, gearbox, bedplate and yaw bearings

Created by Ryan King 2013.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component, Assembly
from openmdao.main.datatypes.api import Float, Bool, Int, Str, Array
import numpy as np
from math import pi
import algopy
import scipy as scp
import scipy.optimize as opt

# -------------------------------------------------

#TODO: Update hub component - this is the old Sunderland component
class Hub(Component):
    ''' Hub class    
          The Hub class is used to represent the hub component of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''

    # variables
    rootMoment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # parameters
    hubDiameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    bladeNumber = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes hub component 
        
        Parameters
        ----------
        rootMoment : float
          maximum flap-wise root moment for the blade (if == 0, then it is set within compute method) [Nm]
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        hubDiameter : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]
        bladeNumber : int
          Number of wind turbine rotor blades
        
        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]   
        '''

        super(Hub, self).__init__()

    def execute(self):

        # Sunderland method for calculating hub, pitch and spinner cone masses
        hubloadFact      = 1                                     # default is 3 blade rigid hub (should depend on hub type)
        hubgeomFact      = 1                                     # default is 3 blade (should depend on blade number)
        hubcontFact      = 1.5                                   # default is 1.5 for full span pitch control (should depend on control method)
        hubmatldensity   = 7860.0                               # density of hub material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        hubmatlstress    = 371000000.0                                # allowable stress of hub material (N / m^2)

        # Root moment required as input, could be undone
        '''if RootMoment == 0.0:
            RootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided'''
        
        self.mass =50 * hubgeomFact * hubloadFact * hubcontFact * self.bladeNumber * self.rootMoment * (hubmatldensity / hubmatlstress)
                                                            # mass of hub based on Sunderland model
                                                            # 31.4 adapted to 50 to fit 5 MW data

        # calculate mass properties
        if self.hubDiameter == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(self.hubDiameter)
        self.thickness = self.diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant

        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * self.rotorDiameter)
        cm[1]     = 0.0
        cm[2]     = 0.025 * self.rotorDiameter
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

        # derivatives
        d_hubMass_d_rootMoment = 50 * hubgeomFact * hubloadFact * hubcontFact * self.bladeNumber * (hubmatldensity / hubmatlstress)
        d_cm_d_rotorDiameter = np.array([-0.05, 0.0, 0.025])
        d_I_d_rootMoment = 0.4 * d_hubMass_d_rootMoment * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter**2
        d_I_d_hubDiameter = 2 * 0.4 * self.mass * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter
        
        # Jacobian
        self.J = np.array([[d_hubMass_d_rootMoment, 0, 0], \
                           [0, d_cm_d_rotorDiameter[0], 0], \
                           [0, d_cm_d_rotorDiameter[1], 0], \
                           [0, d_cm_d_rotorDiameter[2], 0], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter]])

    def provideJ(self):

        input_keys = ['rootMoment', 'rotorDiameter', 'hubDiameter']
        output_keys = ['hubMass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class LowSpeedShaft_drive(Component):
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotorTorque = Float(iotype='in', units='N*m', desc='The torque load due to aerodynamic forces on the rotor')
    rotorBendingMoment = Float(iotype='in', units='N*m', desc='The bending moment from uneven aerodynamic loads')
    rotorMass = Float(iotype='in', units='kg', desc='rotor mass')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorSpeed = Float(iotype='in', units='rpm', desc='rotor speed at rated power')
    machineRating = Float(iotype='in', units='kW', desc='machineRating machine rating of the turbine')

    # parameters
    shaftAngle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaftLength = Float(iotype='in', units='m', desc='length of low speed shaft')
    shaftD1 = Float(iotype='in', units='m', desc='Fraction of LSS distance from gearbox to downwind main bearing')
    shaftD2 = Float(iotype='in', units='m', desc='raction of LSS distance from gearbox to upwind main bearing')
    shaftRatio = Float(0.0, iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    
    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    

    def __init__(self):
        '''
        Initializes low speed shaft component

        Parameters
        ----------
        rotorTorque : float
          The torque load due to aerodynamic forces on the rotor [N*m]
        rotorBendingMoment : float
          The bending moment from uneven aerodynamic loads [N*m]
        rotorMass : float
          The rotor mass [kg]
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorSpeed : float
          The speed of the rotor at rated power [rpm]
        shaftAngle : float
          Angle of the LSS inclindation with respect to the horizontal [deg]
        shaftLength : float
          Length of the LSS [m]
        shaftD1 : float
          Fraction of LSS distance from gearbox to downwind main bearing
        shaftD2 : float
          Fraction of LSS distance from gearbox to upwind main bearing
        machineRating : float
          machineRating power rating for the turbine [W]
        shaftRatio : float
          Ratio of inner diameter to outer diameter.  Leave zero for solid LSS. 

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]   
        '''

        super(LowSpeedShaft, self).__init__()
    
    def execute(self):

        self.mass = self.calc_mass([self.rotorTorque, self.rotorBendingMoment, self.rotorMass, self.rotorDiameter, self.rotorSpeed, \
                                    self.shaftAngle, self.shaftLength, self.shaftD1, self.shaftD2, self.machineRating, self.shaftRatio])
        
        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotorDiameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotorDiameter
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0) / 8.0
        I[1]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0 + (4.0 / 3.0) * (self.length ** 2.0)) / 16.0
        I[2]  = I[1]
        self.I = I

        # derivatives
        x = algopy.UTPM.init_jacobian([self.rotorTorque, self.rotorBendingMoment, self.rotorMass, self.rotorDiameter, self.rotorSpeed, self.machineRating])  

        d_mass = algopy.UTPM.extract_jacobian(self.calc_mass(x))
        d_mass_d_rotorTorque = d_mass[0]
        d_mass_d_rotorBendingMoment = d_mass[1]
        d_mass_d_rotorMass = d_mass[2]   
        d_mass_d_rotorDiameter = d_mass[3]
        d_mass_d_rotorSpeed = d_mass[4]
        d_mass_d_machineRating = d_mass[5] 

        d_cm_d_rotorDiameter = np.array([- (0.035 - 0.01), 0.0, 0.025])

        #TODO: d_I_d_x
        '''d_I_d_rotorDiameter = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorDiameter[0] = (1/8) * (1+ ioratio**2) * d_lssMass_d_rotorDiameter * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotorDiameter
        d_I_d_rotorDiameter[1] = (1/2) * d_I_d_rotorDiameter[0] + (1/16) * (4/3) * 2 * self.length * d_length_d_rotorDiameter
        d_I_d_rotorDiameter[2] = d_I_d_rotorDiameter[1]

        d_I_d_rotorTorque = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorTorque[0] = (1/8) * (1+ ioratio**2) * d_lssMass_d_rotorTorque * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotorTorque
        d_I_d_rotorTorque[1] = (1/2) * d_I_d_rotorTorque[0]
        d_I_d_rotorTorque[2] = d_I_d_rotorTorque[1]

        d_I_d_rotorMass = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorMass[0] = (1/8) * (1+ ioratio**2) * d_lssMass_d_rotorMass * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotorMass
        d_I_d_rotorMass[1] = (1/2) * d_I_d_rotorMass[0]
        d_I_d_rotorMass[2] = d_I_d_rotorMass[1] '''
    
    def calc_mass(self, x):
    
        [rotorTorque, rotorBendingMoment, rotorMass, rotorDiaemeter, rotorSpeed, shaftAngle, shaftLength, shaftD1, shaftD2, machineRating, shaftRatio] = x

        #torque check
        if rotorTorque == 0:
            omega=rotorSpeed/60*(2*pi)      #rotational speed in rad/s at rated power
            eta=0.944                 #drivetrain efficiency
            rotorTorque=machineRating/(omega*eta)         #torque

        self.length=shaftLength
            
        # compute masses, dimensions and cost
        #static overhanging rotor moment (need to adjust for CM of rotor not just distance to end of LSS)
        L2=shaftLength*shaftD2                   #main bearing to end of mainshaft
        alpha=shaftAngle*pi/180.0           #shaft angle
        L2=L2*cos(alpha)                  #horizontal distance from main bearing to hub center of mass
        staticRotorMoment=rotorMass*L2*9.81      #static bending moment from rotor
      
        #assuming 38CrMo4 / AISI 4140 from http://www.efunda.com/materials/alloys/alloy_steels/show_alloy.cfm?id=aisi_4140&prop=all&page_title=aisi%204140
        yieldStrength=417.0*10.0**6 #Pa
        steelDensity=8.0*10.0**3
        
        #Safety Factors
        gammaAero=1.35
        gammaGravity=1.35 #some talk of changing this to 1.1
        gammaFavorable=0.9
        gammaMaterial=1.25 #most conservative
        
        maxFactoredStress=yieldStrength/gammaMaterial
        factoredrotorTorque=rotorTorque*gammaAero
        factoredTotalRotorMoment=rotorBendingMoment*gammaAero-staticRotorMoment*gammaFavorable

        self.D_outer=outerDiameterStrength(shaftRatio,maxFactoredStress)
        self.D_inner=shaftRatio*self.D_outer

        #print "LSS outer diameter is %f m, inner diameter is %f m" %(self.D_outer, self.D_inner)
        
        J=Jmoment(self.D_outer,self.D_inner)
        I=Imoment(self.D_outer,self.D_inner)
        
        sigmaX=bendingStress(factoredTotalRotorMoment, self.D_outer/2.0, I)
        tau=shearStress(rotorTorque, self.D_outer/2.0, J)
        
        #print "Max unfactored normal bending stress is %g MPa" % (sigmaX/1.0e6)
        #print "Max unfactored shear stress is %g MPa" % (tau/1.0e6)
        
        volumeLSS=((self.D_outer/2.0)**2.0-(self.D_inner/2.0)**2.0)*pi*shaftLength
        mass=volumeLSS*steelDensity
        
        return mass
        
		    # Second moment of area for hollow shaft
		    def Imoment(d_o,d_i):
		        I=(pi/64.0)*(d_o**4-d_i**4)
		        return I
		    
		    # Second polar moment for hollow shaft
		    def Jmoment(d_o,d_i):
		        J=(pi/32.0)*(d_o**4-d_i**4)
		        return J
		    
		    # Bending stress
		    def bendingStress(M, y, I):
		        sigma=M*y/I
		        return sigma
		    
		    # Shear stress
		    def shearStress(T, r, J):
		        tau=T*r/J
		        return tau
		    
		    #Find the necessary outer diameter given a diameter ratio and max stress
		    def outerDiameterStrength(shaftRatio,maxFactoredStress):
		        D_outer=(16.0/(pi*(1.0-shaftRatio**4.0)*maxFactoredStress)*(factoredTotalRotorMoment+sqrt(factoredTotalRotorMoment**2.0+factoredrotorTorque**2.0)))**(1.0/3.0)
		        return D_outer

    def provideJ(self):

        #TODO: provideJ update
        '''input_keys = ['rotorDiameter', 'rotorTorque', 'rotorMass']
        output_keys = ['length', 'designTorque', 'designBendingLoad', 'mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)'''


#-------------------------------------------------------------------------------

class MainBearings(Component):
    ''' MainBearings class
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def __init__(self, shaftD1, shaftD2, shaftLength, rotorMass, rotorBendingMoment, D_outer):
        ''' Initializes main bearings component

        Parameters
        ----------
        shaftD1 : float
          Fraction of LSS length from gearbox to downwind main bearing.
        shaftD2 : float
          Fraction of LSS length from gearbox to upwind main bearing.
        shaftLength : float
          Length of the LSS [m].
        rotorMass : float
          Mass of the rotor [kg].
        rotorBendingMoment : float
          Aerodynamic bending moment [N*m].
        D_outer : float
          Outer diameter of the LSS [m].
        '''

        self.upwindBearing = Bearing()
        self.downwindBearing = Bearing()


        #compute reaction forces
        #Safety Factors
        gammaAero=1.35
        gammaGravity=1.35 #some talk of changing this to 1.1
        gammaFavorable=0.9
        gammaMaterial=1.25 #most conservative
        
        #Bearing 1 is closest to gearbox, Bearing 2 is closest to rotor
        L2=shaftLength*shaftD2
        L1=shaftLength*shaftD1

        Fstatic=rotorMass*9.81*gammaFavorable #N
        Mrotor=rotorBendingMoment*gammaAero #Nm

        R2=(-Mrotor+Fstatic*shaftD1)/(shaftD2-shaftD1)
        #print "R2: %g" %(R2)

        R1=-Fstatic-R2
        #print "R1: %g" %(R1)
        
        

        # compute masses, dimensions and cost

        self.upwindBearing.mass = 485.0
        self.downwindBearing.mass = 460.0

        self.mass = (self.upwindBearing.mass + self.downwindBearing.mass)

        # # calculate mass properties
        # inDiam  = lss.diameter
        # self.depth = (inDiam * 1.5)

        # cmMB = np.array([0.0,0.0,0.0])
        # cmMB = ([- (0.035 * rotorDiameter), 0.0, 0.025 * rotorDiameter])
        # self.mainBearing.cm = cmMB

        # cmSB = np.array([0.0,0.0,0.0])
        # cmSB = ([- (0.01 * rotorDiameter), 0.0, 0.025 * rotorDiameter])
        # self.secondBearing.cm = cmSB

        # cm = np.array([0.0,0.0,0.0])
        # for i in (range(0,3)):
        #     # calculate center of mass
        #     cm[i] = (self.mainBearing.mass * self.mainBearing.cm[i] + self.secondBearing.mass * self.secondBearing.cm[i]) \
        #               / (self.mainBearing.mass + self.secondBearing.mass)
        # self.cm = cm

        # self.b1I0 = (b1mass * inDiam ** 2 ) / 4 + (h1mass * self.depth ** 2) / 4
        # self.mainBearing.I = ([self.b1I0, self.b1I0 / 2, self.b1I0 / 2])

        # self.b2I0  = (b2mass * inDiam ** 2 ) / 4 + (h2mass * self.depth ** 2) / 4
        # self.secondBearing.I = ([self.b2I0, self.b2I0 / 2, self.b2I0 / 2])

        # I = np.array([0.0, 0.0, 0.0])
        # for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
        #     # calculate moments around CM
        #     # sum moments around each components CM
        #     I[i]  =  self.mainBearing.I[i] + self.secondBearing.I[i]
        #     # translate to nacelle CM using parallel axis theorem
        #     for j in (range(0,3)):
        #         if i != j:
        #             I[i] +=  (self.mainBearing.mass * (self.mainBearing.cm[i] - self.cm[i]) ** 2) + \
        #                           (self.secondBearing.mass * (self.secondBearing.cm[i] - self.cm[i]) ** 2)
        # self.I = I

#-------------------------------------------------------------------------------

class Gearbox(Component):
    ''' Gearbox class
          The Gearbox class is used to represent the gearbox component of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def __init__(self, name, gbxPower, ratio, gearConfig, Np, rotorSpeed, eff, ratioType='optimal', shType='normal'):
        '''
        Initializes gearbox component

        Parameters
        ----------
        name : str
          Name of the gearbox.
        power : float
          Rated power of the gearbox [W].
        ratio : float
          Overall gearbox speedup ratio.
        gearConfig : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.  'eep_3' and 'eep_2 are also options that fix the final stage ratio at 3 or 2 respectively.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        rotorSpeed : float
          Rotational speed of the LSS at rated power [rpm].
        eff : float
          Mechanical efficiency of the gearbox.
        ratioType : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shType : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        torque : float
          Gearbox torque rating based off gearbox power rating.
        '''

        self.gbxPower=gbxPower
        self.eff=eff
        self.rotorSpeed=rotorSpeed
        self.gearConfig=gearConfig
        self.ratio=ratio
        self.name=name
        self.Np=Np
        self.ratioType=ratioType
        self.shType=shType

        def rotorTorque():
            tq = self.gbxPower / self.eff / (self.rotorSpeed * (pi / 30.0))
            return tq
        
        self.torque=rotorTorque()

        self.stageRatio=np.zeros([3,1])

        self.stageTorque = np.zeros([len(self.stageRatio),1]) #filled in when ebxWeightEst is called
        self.stageMass = np.zeros([len(self.stageRatio),1]) #filled in when ebxWeightEst is called
        
        def stageTypeCalc(config):
            temp=[]
            for character in config:
                    if character == 'e':
                        temp.append(2)
                    if character == 'p':
                        temp.append(1)
            return temp

        self.stageType=stageTypeCalc(self.gearConfig)


        def stageMassCalc(indStageRatio,indNp,indStageType):

            '''
            Computes the mass of an individual gearbox stage.

            Parameters
            ----------
            indStageRatio : str
              Speedup ratio of the individual stage in question.
            indNp : int
              Number of planets for the individual stage.
            indStageType : int
              Type of gear.  Use '1' for parallel and '2' for epicyclic.
            '''

            #Application factor to include ring/housing/carrier weight
            Kr=0.4

            if indNp == 3:
                Kgamma=1.1
            elif indNp == 4:
                Kgamma=1.25
            elif indNp == 5:
                Kgamma=1.35

            if indStageType == 1:
                indStageMass=1.0+indStageRatio+indStageRatio**2+(1.0/indStageRatio)

            elif indStageType == 2:
                sunRatio=0.5*indStageRatio - 1.0
                indStageMass=Kgamma*((1/indNp)+(1/(indNp*sunRatio))+sunRatio+sunRatio**2+Kr*((indStageRatio-1)**2)/indNp+Kr*((indStageRatio-1)**2)/(indNp*sunRatio))

            return indStageMass
            
        def gbxWeightEst(config,overallRatio,Np,ratioType,shType,torque):


            '''
            Computes the gearbox weight based on a surface durability criteria.
            '''

            ## Define Application Factors ##
            #Application factor for weight estimate
            Ka=0.6
            Kshaft=0.0
            Kfact=0.0

            #K factor for pitting analysis
            if self.torque < 200000.0:
                Kfact = 850.0
            elif self.torque < 700000.0:
                Kfact = 950.0
            else:
                Kfact = 1100.0

            #Unit conversion from Nm to inlb and vice-versa
            Kunit=8.029

            # Shaft length factor
            if self.shType == 'normal':
                Kshaft = 1.0
            elif self.shType == 'short':
                Kshaft = 1.25

            #Individual stage torques
            torqueTemp=self.torque
            for s in range(len(self.stageRatio)):
                self.stageTorque[s]=torqueTemp/self.stageRatio[s]
                torqueTemp=self.stageTorque[s]
                self.stageMass[s]=Kunit*Ka/Kfact*self.stageTorque[s]*stageMassCalc(self.stageRatio[s],self.Np[s],self.stageType[s])
            
            gbxWeight=(sum(self.stageMass))*Kshaft
            
            return gbxWeight




        def stageRatioCalc(overallRatio,Np,ratioType,config):
            '''
            Calculates individual stage ratios using either empirical relationships from the Sunderland model or a SciPy constrained optimization routine.
            '''

            K_r=0
                        
            #Assumes we can model everything w/Sunderland model to estimate speed ratio
            if ratioType == 'empirical':
                if config == 'p': 
                    x=[overallRatio]
                if config == 'e':
                    x=[overallRatio]
                elif config == 'pp':
                    x=[overallRatio**0.5,overallRatio**0.5]
                elif config == 'ep':
                    x=[overallRatio/2.5,2.5]
                elif config =='ee':
                    x=[overallRatio**0.5,overallRatio**0.5]
                elif config == 'eep':
                    x=[(overallRatio/3)**0.5,(overallRatio/3)**0.5,3]
                elif config == 'epp':
                    x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                elif config == 'eee':
                    x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                elif config == 'ppp':
                    x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
            
            elif ratioType == 'optimal':
                x=np.zeros([3,1])

                if self.gearConfig == 'eep':
                    x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                    B_1=Np[0]
                    B_2=Np[1]
                    K_r1=0
                    K_r2=0 #2nd stage structure weight coefficient

                    def volume(x):
                        return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                    
                    def constr1(x,overallRatio):
                        return x[0]*x[1]*x[2]-overallRatio
            
                    def constr2(x,overallRatio):
                        return overallRatio-x[0]*x[1]*x[2]

                    x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7)
            
                elif config == 'eep_3':
                    #fixes last stage ratio at 3
                    x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                    B_1=Np[0]
                    B_2=Np[1]
                    K_r1=0
                    K_r2=0.8 #2nd stage structure weight coefficient

                    def volume(x):
                        return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                    
                    def constr1(x,overallRatio):
                        return x[0]*x[1]*x[2]-overallRatio
            
                    def constr2(x,overallRatio):
                        return overallRatio-x[0]*x[1]*x[2]
                    
                    def constr3(x,overallRatio):
                        return x[2]-3.0
                    
                    def constr4(x,overallRatio):
                        return 3.0-x[2]

                    x=opt.fmin_cobyla(volume, x0,[constr1,constr2,constr3,constr4],consargs=[overallRatio],rhoend=1e-7)
                
                elif config == 'eep_2':
                    #fixes final stage ratio at 2
                    x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                    B_1=Np[0]
                    B_2=Np[1]
                    K_r1=0
                    K_r2=1.6 #2nd stage structure weight coefficient

                    def volume(x):
                        return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                    
                    def constr1(x,overallRatio):
                        return x[0]*x[1]*x[2]-overallRatio
            
                    def constr2(x,overallRatio):
                        return overallRatio-x[0]*x[1]*x[2]

                    x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7)
            
                else:
                    x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                    B_1=Np[0]
                    K_r=0.0
                    def volume(x):
                        return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1)+(x[0]/2.0-1.0)**2+K_r*((x[0]-1.0)**2)/B_1 + K_r*((x[0]-1)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*(1.0+(1.0/x[1])+x[1] + x[1]**2)+ (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)
                                      
                    def constr1(x,overallRatio):
                        return x[0]*x[1]*x[2]-overallRatio
            
                    def constr2(x,overallRatio):
                        return overallRatio-x[0]*x[1]*x[2]

                    x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7)
            else:
                x='fail'

                       
            return x


        self.stageRatio=stageRatioCalc(self.ratio,self.Np,self.ratioType,self.gearConfig)

        self.mass=gbxWeightEst(self.gearConfig,self.ratio,self.Np,self.ratioType,self.shType,self.torque)

        # self.stageMass=stageMass
        # self.stageTorque=stageTorque
        
#---------------------------------------------------------------------------------------------------------------

class Bedplate(Component):
    ''' Bedplate class
          The Bedplate class is used to represent the bedplate of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def __init__(self,towerTopDiam, shaftLength, rotorDiameter, uptowerTransformer=0):
        ''' Initializes bedplate component

        Parameters
        ----------
        towerTopDiam : float
          Diameter of the top tower section at the nacelle flange [m].
        shaftLength : float
          Length of the LSS [m]
        rotorDiameter : float
          The wind turbine rotor diameter [m].
        uptowerTransformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        '''

        #Treat front cast iron main beam in similar manner to hub
        #Use 1/4 volume of annulus given by:
        #length is LSS length plus tower radius
        #diameter is 1.25x tower diameter
        #guess at initial thickness same as hub
        
        castThickness=rotorDiameter/620
        castVolume=(shaftLength+towerTopDiam/2)*pi*(towerTopDiam*1.25)*0.25*castThickness
        castDensity=7.1*10.0**3 #kg/m^3
        castMass=castDensity*castVolume
        self.length=0.0
        self.width=1.2
        self.depth=0.66
        
        #These numbers based off V80, need to update
        steelVolume=0.0
        if uptowerTransformer == 1:
            self.length=1.5*shaftLength
            steelVolume=self.length*self.width*self.depth/4.0
        elif uptowerTransformer == 0:
            self.length=shaftLength
            steelVolume=self.length*self.width*self.depth/4.0
        steelDensity=7900 #kg/m**3
        steelMass=steelDensity*steelVolume

        self.mass = steelMass + castMass

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = cm[1] = 0.0
        cm[2] = 0.0122 * rotorDiameter                             # half distance from shaft to yaw axis
        self.cm = cm

        self.depth = (self.length / 2.0)

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.width ** 2 + self.depth ** 2) / 8
        I[1]  = self.mass * (self.depth ** 2 + self.width ** 2 + (4/3) * self.length ** 2) / 16
        I[2]  = I[1]
        self.I = I

#-------------------------------------------------------------------------------

class YawSystem(Component):
    ''' YawSystem class
          The YawSystem class is used to represent the yaw system of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def __init__(self, towerTopDiam, rotorDiameter, numYawMotors=0):
        ''' Initializes yaw system

        Parameters
        ----------
        towerTopDiam : float
          Diameter of the tower top section [m]
        rotorDiameter : float
          Rotor Diameter [m].
        numYawMotors : int
          Number of yaw motors.
        '''

        self.numYawMotors=numYawMotors

        if self.numYawMotors == 0 :
          if rotorDiameter < 90.0 :
            self.numYawMotors = 4.0
          elif rotorDiameter < 120.0 :
            self.numYawMotors = 6.0
          else:
            self.numYawMotors = 8.0


        
        #assume friction plate surface width is 1/10 the diameter
        #assume friction plate thickness scales with rotor diameter
        frictionPlateVol=pi*towerTopDiam*(towerTopDiam*0.10)*(rotorDiameter/1000.0)
        steelDensity=8000.0
        frictionPlateMass=frictionPlateVol*steelDensity
        
        #Assume same yaw motors as Vestas V80 for now: Bonfiglioli 709T2M
        yawMotorMass=190.0
        
        totalYawMass=frictionPlateMass + (numYawMotors*yawMotorMass)
        self.mass= totalYawMass

        # calculate mass properties
        # yaw system assumed to be collocated to tower top center
        cm = np.array([0.0,0.0,0.0])
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        self.I = I

if __name__ == '__main__':
     pass
