klampt.robotsim (core classes) module
=====================================

The robotsim module contains all of the core classes and functions from the C++ API.  These are imported into the main ``klampt`` namespace.

Note: The C++ API is converted from SWIG, so the documentation may be a little rough. The first lines of the documentation for overloaded SWIG functions may describe the signature for each function overload.  For example, :meth:`WorldModel.add` contains the listing::

    add (name,robot): RobotModel
    add (name,obj): RigidObjectModel
    add (name,terrain): TerrainModel

    Parameters: * name (str) –
                * robot (RobotModel, optional) –
                * obj (RigidObjectModel, optional) –
                * terrain (TerrainModel, optional) –

The colon followed by a type descriptor, ``: Type``, gives the type of the return value.  This  means that if the second argument is a RobotModel, the first overload is matched, and the return value is a :class:`RobotModel`.

Modeling robots and worlds
--------------------------

.. autosummary::
    ~klampt.WorldModel
    ~klampt.RobotModel
    ~klampt.RobotModelLink
    ~klampt.RobotModelDriver
    ~klampt.RigidObjectModel
    ~klampt.TerrainModel
    ~klampt.Mass
    ~klampt.ContactParameters
    ~klampt.setRandomSeed
    ~klampt.destroy

Modeling geometries
-------------------

.. autosummary::
    ~klampt.Geometry3D
    ~klampt.Appearance
    ~klampt.GeometricPrimitive
    ~klampt.TriangleMesh
    ~klampt.PointCloud
    ~klampt.VolumeGrid
    ~klampt.DistanceQuerySettings
    ~klampt.DistanceQueryResult

Inverse kinematics
-------------------

.. autosummary::
    ~klampt.IKObjective
    ~klampt.IKSolver
    ~klampt.GeneralizedIKObjective
    ~klampt.GeneralizedIKSolver
    ~klampt.SampleTransform

Simulation
-------------------

.. autosummary::
    ~klampt.Simulator
    ~klampt.SimBody
    ~klampt.SimRobotController
    ~klampt.SimRobotSensor

Equilibrium testing
-------------------

.. autosummary::
    ~klampt.comEquilibrium
    ~klampt.comEquilibrium2D
    ~klampt.equilibriumTorques
    ~klampt.forceClosure
    ~klampt.forceClosure2D
    ~klampt.setFrictionConeApproximationEdges
    ~klampt.supportPolygon
    ~klampt.supportPolygon2D

Input/Output and Visualization
------------------------------

.. autosummary::
    ~klampt.SubscribeToStream
    ~klampt.DetachFromStream
    ~klampt.ProcessStreams
    ~klampt.WaitForStream
    ~klampt.ThreeJSGetScene
    ~klampt.ThreeJSGetTransforms
    ~klampt.Widget
    ~klampt.WidgetSet
    ~klampt.ObjectPoser
    ~klampt.RobotPoser
    ~klampt.PointPoser
    ~klampt.TransformPoser
    ~klampt.Viewport

Module contents
----------------

.. automodule:: klampt
    :members:
    :undoc-members:
    :show-inheritance:
    :exclude-members: doubleArray, floatArray, doubleVector, intArray, stringArray