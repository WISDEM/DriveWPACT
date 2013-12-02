"""
nacelleSE.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component, Assembly
from openmdao.main.datatypes.api import Float, Bool, Int, Str, Array
from math import pi, cos, sqrt
import numpy as np

from NacelleSE_components import LowSpeedShaft, MainBearing, SecondBearing, Gearbox, HighSpeedSide, Generator, Bedplate, AboveYawMassAdder, YawSystem, NacelleSystemAdder
from DriveSE_components import LowSpeedShaft_drive, Gearbox_drive, MainBearing_drive, SecondBearing_drive, Bedplate_drive,YawSystem_drive, LowSpeedShaft_drive3pt, LowSpeedShaft_drive4pt

class NacelleBase(Assembly):

    # variables
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorMass = Float(iotype='in', units='kg', desc='rotor mass')
    rotorTorque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    rotorThrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    rotorSpeed = Float(iotype='in', units='m/s', desc='rotor speed at rated')
    machineRating = Float(iotype='in', units='kW', desc='machine rating of generator')
    gearRatio = Float(iotype='in', desc='overall gearbox ratio')
    towerTopDiameter = Float(iotype='in', units='m', desc='diameter of tower top')
    rotorBendingMoment = Float(iotype='in', units='N*m', desc='maximum aerodynamic bending moment')

    # parameters
    drivetrainDesign = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive')
    crane = Bool(iotype='in', desc='flag for presence of crane')
    bevel = Int(iotype='in', desc='Flag for the presence of a bevel stage - 1 if present, 0 if not')
    gearConfiguration = Str(iotype='in', desc='tring that represents the configuration of the gearbox (stage number and types)')

#-------------------------------------------------------------------------------------

class NacelleSE(NacelleBase):
    ''' 
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.             
    '''

    def configure(self):

        # select components
        self.add('aboveYawMassAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft())
        self.add('mainBearing', MainBearing())
        self.add('secondBearing',SecondBearing())
        self.add('gearbox', Gearbox())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate())
        self.add('yawSystem', YawSystem())
        
        # workflow
        self.driver.workflow.add(['aboveYawMassAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])
        
        # connect inputs
        self.connect('rotorDiameter', ['lowSpeedShaft.rotorDiameter', 'mainBearing.rotorDiameter', 'secondBearing.rotorDiameter', 'gearbox.rotorDiameter', 'highSpeedSide.rotorDiameter', \
                     'generator.rotorDiameter', 'bedplate.rotorDiameter', 'yawSystem.rotorDiameter'])
        self.connect('rotorTorque', ['lowSpeedShaft.rotorTorque', 'gearbox.rotorTorque', 'highSpeedSide.rotorTorque', 'bedplate.rotorTorque'])
        self.connect('rotorMass', ['lowSpeedShaft.rotorMass', 'bedplate.rotorMass'])
        self.connect('rotorSpeed', ['mainBearing.rotorSpeed', 'secondBearing.rotorSpeed'])
        self.connect('rotorThrust', ['bedplate.rotorThrust', 'yawSystem.rotorThrust'])
        self.connect('towerTopDiameter', ['bedplate.towerTopDiameter', 'yawSystem.towerTopDiameter'])
        self.connect('machineRating', ['generator.machineRating', 'aboveYawMassAdder.machineRating'])
        self.connect('drivetrainDesign', ['gearbox.drivetrainDesign', 'generator.drivetrainDesign', 'bedplate.drivetrainDesign'])
        self.connect('gearRatio', ['gearbox.gearRatio', 'generator.gearRatio', 'highSpeedSide.gearRatio'])
        self.connect('gearConfiguration', 'gearbox.gearConfiguration')
        self.connect('bevel', 'gearbox.bevel')
        self.connect('crane', 'aboveYawMassAdder.crane')
        
        
        # connect components
        self.connect('lowSpeedShaft.designTorque', ['mainBearing.lssDesignTorque', 'secondBearing.lssDesignTorque'])
        self.connect('lowSpeedShaft.diameter', ['mainBearing.lssDiameter', 'secondBearing.lssDiameter', 'highSpeedSide.lssDiameter'])
        self.connect('bedplate.length', 'aboveYawMassAdder.bedplateLength')
        self.connect('bedplate.width', 'aboveYawMassAdder.bedplateWidth')

        self.connect('lowSpeedShaft.mass', ['mainBearing.lowSpeedShaftMass', 'secondBearing.lowSpeedShaftMass', 'aboveYawMassAdder.lowSpeedShaftMass', 'nacelleSystem.lowSpeedShaftMass'])
        self.connect('mainBearing.mass', ['aboveYawMassAdder.mainBearingMass', 'nacelleSystem.mainBearingMass'])
        self.connect('secondBearing.mass', ['aboveYawMassAdder.secondBearingMass', 'nacelleSystem.secondBearingMass'])
        self.connect('gearbox.mass', ['aboveYawMassAdder.gearboxMass', 'nacelleSystem.gearboxMass'])
        self.connect('highSpeedSide.mass', ['aboveYawMassAdder.highSpeedSideMass', 'nacelleSystem.highSpeedSideMass'])
        self.connect('generator.mass', ['aboveYawMassAdder.generatorMass', 'nacelleSystem.generatorMass'])
        self.connect('bedplate.mass', ['aboveYawMassAdder.bedplateMass', 'nacelleSystem.bedplateMass'])
        self.connect('aboveYawMassAdder.mainframeMass', 'nacelleSystem.mainframeMass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('aboveYawMassAdder.aboveYawMass', ['yawSystem.aboveYawMass', 'nacelleSystem.aboveYawMass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lowSpeedShaftCM'])
        self.connect('mainBearing.cm', 'nacelleSystem.mainBearingCM')
        self.connect('secondBearing.cm', 'nacelleSystem.secondBearingCM')
        self.connect('gearbox.cm', ['nacelleSystem.gearboxCM'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.highSpeedSideCM'])
        self.connect('generator.cm', ['nacelleSystem.generatorCM'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplateCM'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lowSpeedShaftI'])
        self.connect('mainBearing.I', 'nacelleSystem.mainBearingI')
        self.connect('secondBearing.I', 'nacelleSystem.secondBearingI')
        self.connect('gearbox.I', ['nacelleSystem.gearboxI'])
        self.connect('highSpeedSide.I', ['nacelleSystem.highSpeedSideI'])
        self.connect('generator.I', ['nacelleSystem.generatorI'])
        self.connect('bedplate.I', ['nacelleSystem.bedplateI'])
            
        # create passthroughs
        self.create_passthrough('nacelleSystem.mass')
        self.create_passthrough('nacelleSystem.cm')
        self.create_passthrough('nacelleSystem.I')
        

    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses
        
        detailedMasses : array_like of float
           detailed masses for nacelle components
        """
        
        detailedMasses = [self.lowSpeedShaftMass, self.mainBearingsMass, self.gearboxMass, self.highSpeedSideMass, self.generatorMass, self.vspdEtronicsMass, \
                self.econnectionsMass, self.hydrCoolingMass, \
                self.ControlsMass, self.yawMass, self.mainframeMass, self.nacelleCovMass]

        return detailedMasses'''

#------------------------------------------------------------------

class NacelleSE_drive(NacelleBase):

    ''' 
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.             
    '''

    # parameters
    rotorBendingMoment = Float(iotype='in', units='N*m', desc='rotor aerodynamic bending moment')
    shaftAngle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaftLength = Float(iotype='in', units='m', desc='length of low speed shaft')
    shaftD1 = Float(iotype='in', desc='Fraction of LSS distance from gearbox to downwind main bearing')
    shaftD2 = Float(iotype='in', desc='raction of LSS distance from gearbox to upwind main bearing')
    shaftRatio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    rotorRatedRPM = Float(iotype='in', units='rpm', desc='Speed of rotor at rated power')
    #gbxPower = Float(iotype='in', units='kW', desc='gearbox rated power')
    #eff = Float(iotype='in', desc='Fraction of LSS distance from gearbox to downwind main bearing')
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    ratioType=Str(iotype='in', desc='optimal or empirical stage ratios')
    shType = Str(iotype='in', desc = 'normal or short shaft length')
    uptowerTransformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')


    
    # potential additional new parameters

    '''        name : str
          Name of the gearbox.
        gbxPower : float
          Rated power of the gearbox [W].
        ratio : float
          Overall gearbox speedup ratio.
        gearConfig : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        eff : float
          Mechanical efficiency of the gearbox.
        ratioType : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shType : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        uptowerTransformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        numYawMotors : int
          Number of yaw motors.'''

    def configure(self):

        # select components
        self.add('aboveYawMassAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft_drive())
        self.add('mainBearing', MainBearing())
        self.add('secondBearing',SecondBearing())
        self.add('gearbox', Gearbox())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate_drive())
        self.add('yawSystem', YawSystem_drive())
        
        # workflow
        self.driver.workflow.add(['aboveYawMassAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])
        
        # connect inputs
        self.connect('rotorDiameter', ['lowSpeedShaft.rotorDiameter', 'mainBearing.rotorDiameter', 'secondBearing.rotorDiameter', 'gearbox.rotorDiameter', 'highSpeedSide.rotorDiameter', \
                     'generator.rotorDiameter', 'bedplate.rotorDiameter', 'yawSystem.rotorDiameter'])
        self.connect('rotorBendingMoment', 'lowSpeedShaft.rotorBendingMoment')
    
        self.connect('rotorTorque', ['lowSpeedShaft.rotorTorque', 'gearbox.rotorTorque', 'highSpeedSide.rotorTorque'])
    
        self.connect('rotorMass', 'lowSpeedShaft.rotorMass')
        self.connect('rotorSpeed', ['mainBearing.rotorSpeed', 'secondBearing.rotorSpeed'])

        self.connect('rotorThrust', 'yawSystem.rotorThrust')
        self.connect('towerTopDiameter', ['bedplate.towerTopDiameter', 'yawSystem.towerTopDiameter'])
        self.connect('machineRating', ['generator.machineRating', 'aboveYawMassAdder.machineRating', 'lowSpeedShaft.machineRating'])
      
        self.connect('drivetrainDesign', 'generator.drivetrainDesign')
        self.connect('gearRatio', ['gearbox.gearRatio', 'generator.gearRatio', 'highSpeedSide.gearRatio'])
        self.connect('gearConfiguration', 'gearbox.gearConfiguration')

        self.connect('crane', 'aboveYawMassAdder.crane')
        self.connect('shaftAngle', 'lowSpeedShaft.shaftAngle')
        self.connect('shaftLength', ['lowSpeedShaft.shaftLength', 'bedplate.shaftLength'])
        self.connect('shaftD1', 'lowSpeedShaft.shaftD1')
        self.connect('shaftD2', 'lowSpeedShaft.shaftD2')
        self.connect('shaftRatio', 'lowSpeedShaft.shaftRatio')
        self.connect('rotorRatedRPM', 'lowSpeedShaft.rotorSpeed')
        
        
        # connect components
        self.connect('lowSpeedShaft.designTorque', ['mainBearing.lssDesignTorque', 'secondBearing.lssDesignTorque'])
        self.connect('lowSpeedShaft.diameter', ['mainBearing.lssDiameter', 'secondBearing.lssDiameter', 'highSpeedSide.lssDiameter'])
        self.connect('bedplate.length', 'aboveYawMassAdder.bedplateLength')
        self.connect('bedplate.width', 'aboveYawMassAdder.bedplateWidth')

        self.connect('lowSpeedShaft.mass', ['mainBearing.lowSpeedShaftMass', 'secondBearing.lowSpeedShaftMass', 'aboveYawMassAdder.lowSpeedShaftMass', 'nacelleSystem.lowSpeedShaftMass'])
        self.connect('mainBearing.mass', ['aboveYawMassAdder.mainBearingMass', 'nacelleSystem.mainBearingMass'])
        self.connect('secondBearing.mass', ['aboveYawMassAdder.secondBearingMass', 'nacelleSystem.secondBearingMass'])
        self.connect('gearbox.mass', ['aboveYawMassAdder.gearboxMass', 'nacelleSystem.gearboxMass'])
        self.connect('highSpeedSide.mass', ['aboveYawMassAdder.highSpeedSideMass', 'nacelleSystem.highSpeedSideMass'])
        self.connect('generator.mass', ['aboveYawMassAdder.generatorMass', 'nacelleSystem.generatorMass'])
        self.connect('bedplate.mass', ['aboveYawMassAdder.bedplateMass', 'nacelleSystem.bedplateMass'])
        self.connect('aboveYawMassAdder.mainframeMass', 'nacelleSystem.mainframeMass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('aboveYawMassAdder.aboveYawMass', ['yawSystem.aboveYawMass', 'nacelleSystem.aboveYawMass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lowSpeedShaftCM'])
        self.connect('mainBearing.cm', 'nacelleSystem.mainBearingCM')
        self.connect('secondBearing.cm', 'nacelleSystem.secondBearingCM')
        self.connect('gearbox.cm', ['nacelleSystem.gearboxCM'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.highSpeedSideCM'])
        self.connect('generator.cm', ['nacelleSystem.generatorCM'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplateCM'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lowSpeedShaftI'])
        self.connect('mainBearing.I', 'nacelleSystem.mainBearingI')
        self.connect('secondBearing.I', 'nacelleSystem.secondBearingI')
        self.connect('gearbox.I', ['nacelleSystem.gearboxI'])
        self.connect('highSpeedSide.I', ['nacelleSystem.highSpeedSideI'])
        self.connect('generator.I', ['nacelleSystem.generatorI'])
        self.connect('bedplate.I', ['nacelleSystem.bedplateI'])
            
        # create passthroughs
        self.create_passthrough('nacelleSystem.mass')
        self.create_passthrough('nacelleSystem.cm')
        self.create_passthrough('nacelleSystem.I')
        

    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses
        
        detailedMasses : array_like of float
           detailed masses for nacelle components
        """
        
        detailedMasses = [self.lowSpeedShaftMass, self.mainBearingsMass, self.gearboxMass, self.highSpeedSideMass, self.generatorMass, self.vspdEtronicsMass, \
                self.econnectionsMass, self.hydrCoolingMass, \
                self.ControlsMass, self.yawMass, self.mainframeMass, self.nacelleCovMass]

        return detailedMasses'''

#------------------------------------------------------------------

class NacelleSE_drive3pt(NacelleBase):

    ''' 
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.             
    '''

    # parameters
    rotorBendingMoment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotorBendingMoment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotorBendingMoment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotorForce_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotorForce_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotorForce_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')    
    shaftAngle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaftRatio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    rotorRatedRPM = Float(iotype='in', units='rpm', desc='Speed of rotor at rated power')
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    ratioType=Str(iotype='in', desc='optimal or empirical stage ratios')
    shType = Str(iotype='in', desc = 'normal or short shaft length')
    uptowerTransformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')
    shrinkDiscMass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')

    
    # potential additional new parameters

    '''        name : str
          Name of the gearbox.
        gbxPower : float
          Rated power of the gearbox [W].
        ratio : float
          Overall gearbox speedup ratio.
        gearConfig : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        eff : float
          Mechanical efficiency of the gearbox.
        ratioType : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shType : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        uptowerTransformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        numYawMotors : int
          Number of yaw motors.'''

    def configure(self):

        # select components
        self.add('aboveYawMassAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft_drive3pt())
        self.add('mainBearing', MainBearing())
        self.add('secondBearing',SecondBearing())
        self.add('gearbox', Gearbox())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate_drive())
        self.add('yawSystem', YawSystem_drive())
        
        # workflow
        self.driver.workflow.add(['aboveYawMassAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])
        
        # connect inputs
        self.connect('rotorDiameter', ['lowSpeedShaft.rotorDiameter', 'mainBearing.rotorDiameter', 'secondBearing.rotorDiameter', 'gearbox.rotorDiameter', 'highSpeedSide.rotorDiameter', \
                     'generator.rotorDiameter', 'bedplate.rotorDiameter', 'yawSystem.rotorDiameter'])
        self.connect('rotorBendingMoment_x', 'lowSpeedShaft.rotorBendingMoment_x')
        self.connect('rotorBendingMoment_y', 'lowSpeedShaft.rotorBendingMoment_y')
        self.connect('rotorBendingMoment_z', 'lowSpeedShaft.rotorBendingMoment_z')
        self.connect('rotorForce_x', 'lowSpeedShaft.rotorForce_x')
        self.connect('rotorForce_y', 'lowSpeedShaft.rotorForce_y')
        self.connect('rotorForce_z', 'lowSpeedShaft.rotorForce_z')
        self.connect('rotorTorque', ['gearbox.rotorTorque', 'highSpeedSide.rotorTorque'])
        self.connect('rotorMass', 'lowSpeedShaft.rotorMass')
        self.connect('rotorSpeed', ['mainBearing.rotorSpeed', 'secondBearing.rotorSpeed'])
        self.connect('rotorThrust', 'yawSystem.rotorThrust')
        self.connect('towerTopDiameter', ['bedplate.towerTopDiameter', 'yawSystem.towerTopDiameter'])
        self.connect('machineRating', ['generator.machineRating', 'aboveYawMassAdder.machineRating', 'lowSpeedShaft.machineRating'])
        self.connect('drivetrainDesign', 'generator.drivetrainDesign')
        self.connect('gearRatio', ['gearbox.gearRatio', 'generator.gearRatio', 'highSpeedSide.gearRatio'])
        self.connect('gearConfiguration', 'gearbox.gearConfiguration')
        self.connect('crane', 'aboveYawMassAdder.crane')
        self.connect('shaftAngle', 'lowSpeedShaft.shaftAngle')
        self.connect('shaftRatio', 'lowSpeedShaft.shaftRatio')
        self.connect('shrinkDiscMass', 'lowSpeedShaft.shrinkDiscMass')
        self.connect('carrierMass', 'lowSpeedShaft.carrierMass')
        
        # connect components
        self.connect('lowSpeedShaft.designTorque', ['mainBearing.lssDesignTorque', 'secondBearing.lssDesignTorque'])
        self.connect('lowSpeedShaft.diameter', ['mainBearing.lssDiameter', 'secondBearing.lssDiameter', 'highSpeedSide.lssDiameter'])
        self.connect('lowSpeedShaft.length', 'bedplate.shaftLength')
        self.connect('bedplate.length', 'aboveYawMassAdder.bedplateLength')
        self.connect('bedplate.width', 'aboveYawMassAdder.bedplateWidth')

        self.connect('lowSpeedShaft.mass', ['mainBearing.lowSpeedShaftMass', 'secondBearing.lowSpeedShaftMass', 'aboveYawMassAdder.lowSpeedShaftMass', 'nacelleSystem.lowSpeedShaftMass'])
        self.connect('mainBearing.mass', ['aboveYawMassAdder.mainBearingMass', 'nacelleSystem.mainBearingMass'])
        self.connect('secondBearing.mass', ['aboveYawMassAdder.secondBearingMass', 'nacelleSystem.secondBearingMass'])
        self.connect('gearbox.mass', ['lowSpeedShaft.gbxMass', 'aboveYawMassAdder.gearboxMass', 'nacelleSystem.gearboxMass'])
        self.connect('highSpeedSide.mass', ['aboveYawMassAdder.highSpeedSideMass', 'nacelleSystem.highSpeedSideMass'])
        self.connect('generator.mass', ['aboveYawMassAdder.generatorMass', 'nacelleSystem.generatorMass'])
        self.connect('bedplate.mass', ['aboveYawMassAdder.bedplateMass', 'nacelleSystem.bedplateMass'])
        self.connect('aboveYawMassAdder.mainframeMass', 'nacelleSystem.mainframeMass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('aboveYawMassAdder.aboveYawMass', ['yawSystem.aboveYawMass', 'nacelleSystem.aboveYawMass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lowSpeedShaftCM'])
        self.connect('mainBearing.cm', 'nacelleSystem.mainBearingCM')
        self.connect('secondBearing.cm', 'nacelleSystem.secondBearingCM')
        self.connect('gearbox.cm', ['nacelleSystem.gearboxCM'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.highSpeedSideCM'])
        self.connect('generator.cm', ['nacelleSystem.generatorCM'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplateCM'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lowSpeedShaftI'])
        self.connect('mainBearing.I', 'nacelleSystem.mainBearingI')
        self.connect('secondBearing.I', 'nacelleSystem.secondBearingI')
        self.connect('gearbox.I', ['nacelleSystem.gearboxI'])
        self.connect('highSpeedSide.I', ['nacelleSystem.highSpeedSideI'])
        self.connect('generator.I', ['nacelleSystem.generatorI'])
        self.connect('bedplate.I', ['nacelleSystem.bedplateI'])
            
        # create passthroughs
        self.create_passthrough('nacelleSystem.mass')
        self.create_passthrough('nacelleSystem.cm')
        self.create_passthrough('nacelleSystem.I')
        

    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses
        
        detailedMasses : array_like of float
           detailed masses for nacelle components
        """
        
        detailedMasses = [self.lowSpeedShaftMass, self.mainBearingsMass, self.gearboxMass, self.highSpeedSideMass, self.generatorMass, self.vspdEtronicsMass, \
                self.econnectionsMass, self.hydrCoolingMass, \
                self.ControlsMass, self.yawMass, self.mainframeMass, self.nacelleCovMass]

        return detailedMasses'''

#------------------------------------------------------------------

class NacelleSE_drive4pt(NacelleBase):

    ''' 
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.             
    '''

    # variables
    rotorBendingMoment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotorBendingMoment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotorBendingMoment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotorForce_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotorForce_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotorForce_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')    
    shaftAngle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal') # Bedplate tilting angle
    shaftRatio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    rotorRatedRPM = Float(iotype='in', units='rpm', desc='Speed of rotor at rated power')

    # parameters
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    ratioType=Str(iotype='in', desc='optimal or empirical stage ratios')
    shType = Str(iotype='in', desc = 'normal or short shaft length')
    uptowerTransformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')
    shrinkDiscMass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')
    carrierMass = Float(iotype='in', units='kg', desc='Carrier mass')
    mb1Type = Str(iotype='in',desc='Main bearing type: CARB, TRB or SRB')
    mb2Type = Str(iotype='in',desc='Second bearing type: CARB, TRB or SRB')

    '''        name : str
          Name of the gearbox.
        gbxPower : float
          Rated power of the gearbox [W].
        ratio : float
          Overall gearbox speedup ratio.
        gearConfig : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        eff : float
          Mechanical efficiency of the gearbox.
        ratioType : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shType : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        uptowerTransformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        numYawMotors : int
          Number of yaw motors.'''

    def configure(self):

        # select components
        self.add('aboveYawMassAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft_drive4pt())
        self.add('mainBearing', MainBearing_drive())
        self.add('secondBearing',SecondBearing_drive())
        self.add('gearbox', Gearbox_drive())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate_drive())
        self.add('yawSystem', YawSystem_drive())
        
        # workflow
        self.driver.workflow.add(['aboveYawMassAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])
        
        # connect inputs
        self.connect('rotorDiameter', ['lowSpeedShaft.rotorDiameter', 'mainBearing.rotor_diameter', 'secondBearing.rotor_diameter', 'gearbox.rotorDiameter', 'highSpeedSide.rotorDiameter', \
                     'generator.rotorDiameter', 'bedplate.rotorDiameter', 'yawSystem.rotorDiameter'])
        self.connect('rotorBendingMoment_x', 'lowSpeedShaft.rotorBendingMoment_x')
        self.connect('rotorBendingMoment_y', 'lowSpeedShaft.rotorBendingMoment_y')
        self.connect('rotorBendingMoment_z', 'lowSpeedShaft.rotorBendingMoment_z')
        self.connect('rotorForce_x', 'lowSpeedShaft.rotorForce_x')
        self.connect('rotorForce_y', 'lowSpeedShaft.rotorForce_y')
        self.connect('rotorForce_z', 'lowSpeedShaft.rotorForce_z')
        self.connect('rotorTorque', ['gearbox.rotorTorque', 'highSpeedSide.rotorTorque'])
        self.connect('rotorMass', 'lowSpeedShaft.rotorMass')
        self.connect('rotorThrust', 'yawSystem.rotorThrust')
        self.connect('towerTopDiameter', ['bedplate.towerTopDiameter', 'yawSystem.towerTopDiameter'])
        self.connect('machineRating', ['generator.machineRating', 'aboveYawMassAdder.machineRating', 'lowSpeedShaft.machineRating'])
        self.connect('drivetrainDesign', 'generator.drivetrainDesign')
        self.connect('gearRatio', ['gearbox.gearRatio', 'generator.gearRatio', 'highSpeedSide.gearRatio'])
        self.connect('gearConfiguration', 'gearbox.gearConfiguration')
        self.connect('crane', 'aboveYawMassAdder.crane')
        self.connect('shaftAngle', 'lowSpeedShaft.shaftAngle')
        self.connect('shaftRatio', 'lowSpeedShaft.shaftRatio')
        self.connect('shrinkDiscMass', 'lowSpeedShaft.shrinkDiscMass')
        self.connect('carrierMass', 'lowSpeedShaft.carrierMass')
        self.connect('mb1Type', ['mainBearing.bearing_type', 'lowSpeedShaft.mb1Type'])
        self.connect('mb2Type', ['secondBearing.bearing_type', 'lowSpeedShaft.mb2Type'])
        self.connect('Np', 'gearbox.Np')
        self.connect('ratioType', 'gearbox.ratioType')
        self.connect('shType', 'gearbox.shType')
        
        # connect components
        self.connect('lowSpeedShaft.designTorque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        self.connect('lowSpeedShaft.diameter1', ['mainBearing.lss_diameter', 'highSpeedSide.lssDiameter'])
        self.connect('lowSpeedShaft.diameter2', 'secondBearing.lss_diameter')
        self.connect('lowSpeedShaft.length', 'bedplate.shaftLength')
        self.connect('bedplate.length', 'aboveYawMassAdder.bedplateLength')
        self.connect('bedplate.width', 'aboveYawMassAdder.bedplateWidth')

        self.connect('lowSpeedShaft.mass', ['aboveYawMassAdder.lowSpeedShaftMass', 'nacelleSystem.lowSpeedShaftMass'])
        self.connect('mainBearing.mass', ['aboveYawMassAdder.mainBearingMass', 'nacelleSystem.mainBearingMass'])
        self.connect('secondBearing.mass', ['aboveYawMassAdder.secondBearingMass', 'nacelleSystem.secondBearingMass'])
        self.connect('gearbox.mass', ['lowSpeedShaft.gbxMass', 'aboveYawMassAdder.gearboxMass', 'nacelleSystem.gearboxMass'])
        self.connect('highSpeedSide.mass', ['aboveYawMassAdder.highSpeedSideMass', 'nacelleSystem.highSpeedSideMass'])
        self.connect('generator.mass', ['aboveYawMassAdder.generatorMass', 'nacelleSystem.generatorMass'])
        self.connect('bedplate.mass', ['aboveYawMassAdder.bedplateMass', 'nacelleSystem.bedplateMass'])
        self.connect('aboveYawMassAdder.mainframeMass', 'nacelleSystem.mainframeMass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('aboveYawMassAdder.aboveYawMass', ['yawSystem.aboveYawMass', 'nacelleSystem.aboveYawMass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lowSpeedShaftCM'])
        self.connect('mainBearing.cm', 'nacelleSystem.mainBearingCM')
        self.connect('secondBearing.cm', 'nacelleSystem.secondBearingCM')
        self.connect('gearbox.cm', ['nacelleSystem.gearboxCM'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.highSpeedSideCM'])
        self.connect('generator.cm', ['nacelleSystem.generatorCM'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplateCM'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lowSpeedShaftI'])
        self.connect('mainBearing.I', 'nacelleSystem.mainBearingI')
        self.connect('secondBearing.I', 'nacelleSystem.secondBearingI')
        self.connect('gearbox.I', ['nacelleSystem.gearboxI'])
        self.connect('highSpeedSide.I', ['nacelleSystem.highSpeedSideI'])
        self.connect('generator.I', ['nacelleSystem.generatorI'])
        self.connect('bedplate.I', ['nacelleSystem.bedplateI'])
            
        # create passthroughs
        self.create_passthrough('nacelleSystem.mass')
        self.create_passthrough('nacelleSystem.cm')
        self.create_passthrough('nacelleSystem.I')
        

    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses
        
        detailedMasses : array_like of float
           detailed masses for nacelle components
        """
        
        detailedMasses = [self.lowSpeedShaftMass, self.mainBearingsMass, self.gearboxMass, self.highSpeedSideMass, self.generatorMass, self.vspdEtronicsMass, \
                self.econnectionsMass, self.hydrCoolingMass, \
                self.ControlsMass, self.yawMass, self.mainframeMass, self.nacelleCovMass]

        return detailedMasses'''

#------------------------------------------------------------------

def example():

    # test of module for turbine data set

    # NREL 5 MW Rotor Variables
    print '----- NREL 5 MW Turbine -----'
    nace = NacelleSE_drive4pt()
    nace.rotorDiameter = 126.0 # m
    nace.rotorSpeed = 11.05 # m/s
    nace.rotorTorque = 6.35e6 #4365248.74 # Nm
    nace.rotorThrust = 599610.0 #500930.84 # N
    nace.rotorMass = 0.0 #142585.75 # kg
    nace.rotorRatedRPM = 12.1 #rpm
    nace.rotorBendingMoment = 14700000.0 #DLC 1.4
    nace.rotorBendingMoment_x = 3.3e5 #4365248.74
    nace.rotorBendingMoment_y = -16665000.0 #14700000.0
    nace.rotorBendingMoment_z = 2896300.0 #0.0
    nace.rotorForce_x = 599610.0 #500930.84
    nace.rotorForce_y = 186780.0 #0.0
    nace.rotorForce_z = -842710.0 #1e6
    # nace.rotorBendingMoment_x = 4.3268e6
    # nace.rotorBendingMoment_y = -1.0552e6
    # nace.rotorBendingMoment_z = -1.4607e7
    # nace.rotorForce_x = 3.0051e5
    # nace.rotorForce_y = 4.3227e4
    # nace.rotorForce_z = -1.2381e6

    # NREL 5 MW Drivetrain variables
    nace.drivetrainDesign = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machineRating = 5000.0 # kW
    nace.gearRatio = 97.0 # 97:1 as listed in the 5 MW reference document
    nace.gearConfiguration = 'eep' # epicyclic-epicyclic-parallel
    #nace.bevel = 0 # no bevel stage
    nace.crane = True # onboard crane present
    nace.shaftAngle = 5.0 #deg
    nace.shaftLength = 3.383 #m
    nace.shaftD1 = 0.25
    nace.shaftD2 = 0.75
    nace.shaftRatio = 0.10
    nace.Np = [3,3,1]
    nace.ratioType = 'optimal'
    nace.shType = 'normal'
    nace.uptowerTransformer=True
    nace.shrinkDiscMass = 1000.0 # estimated
    nace.carrierMass = 8000.0 # estimated
    nace.mb1Type = 'CARB'
    nace.mb2Type = 'SRB'


    # NREL 5 MW Tower Variables
    nace.towerTopDiameter = 3.78 # m

    nace.run()
    
    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stageMasses[0], nace.gearbox.stageMasses[1], nace.gearbox.stageMasses[2])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.aboveYawMassAdder.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.aboveYawMassAdder.mainframeMass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.aboveYawMassAdder.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.aboveYawMassAdder.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.aboveYawMassAdder.nacelleCovMass , nace.aboveYawMassAdder.height, nace.aboveYawMassAdder.width, nace.aboveYawMassAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg .cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1


def example2():

    # WindPACT 1.5 MW Drivetrain variables
    nace = NacelleSE()
    nace.drivetrainDesign = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machineRating = 1500 # machine rating [kW]
    nace.gearRatio = 87.965
    nace.gearConfiguration = 'epp'
    nace.bevel = 0
    nace.crane = True

    # WindPACT 1.5 MW Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 80 # max tip speed [m/s]
    nace.rotorDiameter = 70 # rotor diameter [m]
    nace.rotorSpeed = 21.830
    DrivetrainEfficiency = 0.95
    nace.rotorTorque = (nace.machineRating * 1000 / DrivetrainEfficiency) / (nace.rotorSpeed * (pi / 30)) 
        # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    nace.rotorThrust = 324000 
    nace.rotorMass = 28560 # rotor mass [kg]

    # WindPACT 1.5 MW Tower Variables
    nace.towerTopDiameter = 2.7 # tower top diameter [m]

    print '----- WindPACT 1.5 MW Turbine -----'
    nace.run()

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stageMasses[1], nace.gearbox.stageMasses[2], nace.gearbox.stageMasses[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.aboveYawMassAdder.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.aboveYawMassAdder.mainframeMass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.aboveYawMassAdder.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.aboveYawMassAdder.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.aboveYawMassAdder.nacelleCovMass , nace.aboveYawMassAdder.height, nace.aboveYawMassAdder.width, nace.aboveYawMassAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg .cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1
 
    # GRC Drivetrain variables
    nace.drivetrainDesign = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machineRating = 750 # machine rating [kW]
    nace.gearRatio = 81.491 
    nace.gearConfiguration = 'epp'
    nace.bevel = 0

    # GRC Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 8 # max tip speed [m/s]
    nace.rotorDiameter = 48.2 # rotor diameter [m]
    #rotorSpeed = MaxTipSpeed / ((rotorDiameter / 2) * (pi / 30)) # max / rated rotor speed [rpm] calculated from max tip speed and rotor diamter
    nace.rotorSpeed = 22
    DrivetrainEfficiency = 0.944
    nace.rotorTorque = (nace.machineRating * 1000 / DrivetrainEfficiency) / (nace.rotorSpeed * (pi / 30)) # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    nace.rotorThrust = 159000 # based on windpact 750 kW design (GRC information not available)
    nace.rotorMass = 13200 # rotor mass [kg]

    # Tower Variables
    nace.towerTopDiameter = 2 # tower top diameter [m] - not given

    print '----- GRC 750 kW Turbine -----'
    nace.run()

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stageMasses[1], nace.gearbox.stageMasses[2], nace.gearbox.stageMasses[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.aboveYawMassAdder.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.aboveYawMassAdder.mainframeMass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.aboveYawMassAdder.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.aboveYawMassAdder.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.aboveYawMassAdder.nacelleCovMass , nace.aboveYawMassAdder.height, nace.aboveYawMassAdder.width, nace.aboveYawMassAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg .cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1

    # Alstom Drivetrain variables
    nace.drivetrainDesign = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machineRating = 3000 # machine rating [kW]
    nace.gearRatio = 102.19 # 
    nace.gearConfiguration = 'eep'
    nace.bevel = 0

    # Alstom Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 80 # max tip speed [m/s]
    nace.rotorDiameter = 100.8 # rotor diameter [m]
    #rotorSpeed = MaxTipSpeed / ((rotorDiameter / 2) * (pi / 30)) # max / rated rotor speed [rpm] calculated from max tip speed and rotor diamter
    nace.rotorSpeed = 15.1 # based on windpact 3 MW
    DrivetrainEfficiency = 0.95
    nace.rotorTorque = (nace.machineRating * 1000 / DrivetrainEfficiency) / (nace.rotorSpeed * (pi / 30)) # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    nace.rotorThrust = 797000 # based on windpact 3.0 MW - Alstom thrust not provided
    nace.rotorMass = 49498 # rotor mass [kg] - not given - using Windpact 3.0 MW

    # Tower Variables
    nace.towerTopDiameter = 3.5 # tower top diameter [m] - not given
    
    print '----- Alstom 3.0 MW Turbine -----'
    nace.run()

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stageMasses[1], nace.gearbox.stageMasses[2], nace.gearbox.stageMasses[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.aboveYawMassAdder.vspdEtronicsMass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.aboveYawMassAdder.mainframeMass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.aboveYawMassAdder.econnectionsMass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.aboveYawMassAdder.hydrCoolingMass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.aboveYawMassAdder.nacelleCovMass , nace.aboveYawMassAdder.height, nace.aboveYawMassAdder.width, nace.aboveYawMassAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg .cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.mass, nace.cm[0], nace.cm[1], nace.cm[2], nace.I[0], nace.I[1], nace.I[2]  )
    # 207727.1


if __name__ == '__main__':
    ''' Main runs through tests of several drivetrain configurations with known component masses and dimensions '''
    
    # todo: adjust to use rotor model interface

    example()
    
    #example2()