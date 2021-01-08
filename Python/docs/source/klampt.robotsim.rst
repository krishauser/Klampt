klampt.robotsim (core classes) module
=====================================

The robotsim module contains all of the core classes and functions from the C++ API.  These are imported into the main ``klampt`` namespace.

Note: The C++ API is converted from SWIG, so the documentation may be a little rough. The first lines of the documentation for overloaded SWIG functions may describe the signature for each function overload.  For example, :meth:`klampt.WorldModel.add` contains the listing::

    add (name,robot): RobotModel
    add (name,obj): RigidObjectModel
    add (name,terrain): TerrainModel

    Parameters: * name (str) –
                * robot (RobotModel, optional) –
                * obj (RigidObjectModel, optional) –
                * terrain (TerrainModel, optional) –

The colon followed by a type descriptor, ``: Type``, gives the type of the return value.  This  means that if the second argument is a RobotModel, the first overload is matched, and the return value is a :class:`klampt.RobotModel`.

Modeling robots and worlds
--------------------------

Imported into the main ``klampt`` package.

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

Imported into the main ``klampt`` package.

.. autosummary::
    ~klampt.Geometry3D
    ~klampt.Appearance
    ~klampt.GeometricPrimitive
    ~klampt.TriangleMesh
    ~klampt.PointCloud
    ~klampt.VolumeGrid
    ~klampt.ConvexHull
    ~klampt.DistanceQuerySettings
    ~klampt.DistanceQueryResult
    ~klampt.ContactQueryResult

Inverse kinematics
-------------------

Imported into the main ``klampt`` package.

.. autosummary::
    ~klampt.IKObjective
    ~klampt.IKSolver
    ~klampt.GeneralizedIKObjective
    ~klampt.GeneralizedIKSolver
    ~klampt.SampleTransform

Simulation
-------------------

Imported into the main ``klampt`` package.

.. autosummary::
    ~klampt.Simulator
    ~klampt.SimBody
    ~klampt.SimJoint
    ~klampt.SimRobotController
    ~klampt.SimRobotSensor

Equilibrium testing
-------------------

See also the aliases in the `klampt.model.contact <klampt.model.contact.html>`__ module.

.. autosummary::
    ~klampt.robotsim.comEquilibrium
    ~klampt.robotsim.comEquilibrium2D
    ~klampt.robotsim.equilibriumTorques
    ~klampt.robotsim.forceClosure
    ~klampt.robotsim.forceClosure2D
    ~klampt.robotsim.setFrictionConeApproximationEdges
    ~klampt.robotsim.supportPolygon
    ~klampt.robotsim.supportPolygon2D

Input/Output
-------------

Imported into the ``klampt.io`` package

.. autosummary::
    ~klampt.io.SubscribeToStream
    ~klampt.io.DetachFromStream
    ~klampt.io.ProcessStreams
    ~klampt.io.WaitForStream
    ~klampt.io.ThreeJSGetScene
    ~klampt.io.ThreeJSGetTransforms

Visualization
--------------

For use in :class:`~klampt.vis.glcommon.GLWidgetProgram`.

.. autosummary::
    ~klampt.robotsim.Widget
    ~klampt.robotsim.WidgetSet
    ~klampt.robotsim.ObjectPoser
    ~klampt.robotsim.RobotPoser
    ~klampt.robotsim.PointPoser
    ~klampt.robotsim.TransformPoser
    ~klampt.robotsim.Viewport

Module contents
----------------

.. automodule:: klampt
    :autosummary:
    :members:
    :undoc-members:
    :show-inheritance:
    :exclude-members: doubleArray, doubleVector, floatArray, floatVector, intArray, intVector, stringArray, stringVector, doubleMatrix

.. autofunction:: klampt.robotsim.setRandomSeed

.. autofunction:: klampt.robotsim.SampleTransform

.. autofunction:: klampt.robotsim.comEquilibrium

.. autofunction:: klampt.robotsim.comEquilibrium2D

.. autofunction:: klampt.robotsim.equilibriumTorques

.. autofunction:: klampt.robotsim.forceClosure

.. autofunction:: klampt.robotsim.forceClosure2D

.. autofunction:: klampt.robotsim.setFrictionConeApproximationEdges

.. autofunction:: klampt.robotsim.supportPolygon

.. autofunction:: klampt.robotsim.supportPolygon2D

.. autoclass:: klampt.robotsim.Widget

.. autoclass:: klampt.robotsim.WidgetSet

.. autoclass:: klampt.robotsim.ObjectPoser

.. autoclass:: klampt.robotsim.RobotPoser

.. autoclass:: klampt.robotsim.PointPoser

.. autoclass:: klampt.robotsim.TransformPoser

.. autoclass:: klampt.robotsim.Viewport
