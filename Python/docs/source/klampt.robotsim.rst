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
    ~klampt.set_random_seed
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
    ~klampt.robotsim.com_equilibrium
    ~klampt.robotsim.com_equilibrium_2d
    ~klampt.robotsim.equilibrium_torques
    ~klampt.robotsim.force_closure
    ~klampt.robotsim.force_closure_2d
    ~klampt.robotsim.set_friction_cone_approximation_edges
    ~klampt.robotsim.support_polygon
    ~klampt.robotsim.support_polygon_2d

Input/Output
-------------

Imported into the ``klampt.io`` package

.. autosummary::
    ~klampt.io.subscribe_to_stream
    ~klampt.io.detach_from_stream
    ~klampt.io.process_streams
    ~klampt.io.wait_for_stream
    ~klampt.io.threejs_get_scene
    ~klampt.io.threejs_get_transforms

Visualization
--------------

For use in :class:`~klampt.vis.glcommon.GLWidgetPlugin`.

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

.. autofunction:: klampt.robotsim.set_random_seed

.. autofunction:: klampt.robotsim.com_equilibrium

.. autofunction:: klampt.robotsim.com_equilibrium_2d

.. autofunction:: klampt.robotsim.equilibrium_torques

.. autofunction:: klampt.robotsim.force_closure

.. autofunction:: klampt.robotsim.force_closure_2d

.. autofunction:: klampt.robotsim.set_friction_cone_approximation_edges

.. autofunction:: klampt.robotsim.support_polygon

.. autofunction:: klampt.robotsim.support_polygon_2d

.. autoclass:: klampt.robotsim.Widget

.. autoclass:: klampt.robotsim.WidgetSet

.. autoclass:: klampt.robotsim.ObjectPoser

.. autoclass:: klampt.robotsim.RobotPoser

.. autoclass:: klampt.robotsim.PointPoser

.. autoclass:: klampt.robotsim.TransformPoser

.. autoclass:: klampt.robotsim.Viewport
