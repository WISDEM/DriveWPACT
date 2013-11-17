"""
common.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component, Assembly
from openmdao.main.datatypes.api import Float, Bool, Int, Array
import numpy as np

class SubComponent(Component):
    ''' 
        Interface for turbine components.  This interface provides a set of attributes for a turbine component.
    '''
    # TODO: harmonization with FUSED_Wind

    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    #diameter = Attribute(""" outer diameter of a cylindrical or spherical component [m] """)
    #length = Attribute(""" y direction == diameter for spheres or vertical cylinders [m] """)
    #width = Attribute(""" x direction == diameter for spheres or veritical cylinders [m] """)
    #height = Attribute("""  z direction == diameter for spheres or horizontal cylinders [m] """)
    #depth = Attribute(""" critical depth on any dimension for component [m] """)
