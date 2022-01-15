Running Klamp't apps
====================================

Klamp't comes with a few utility programs that are installed into your Python/Scripts folder.

- ``klampt_browser``: a program for browsing through and editing resources.
- ``klampt_control``: a program for controlling robots and debugging Robot Interface Layer controllers.
- ``klampt_path``: a command-line utility for modifying paths.
- ``klampt_resource``: a utility program for modifying resources and generating thumbnails.
- ``klampt_sim``: an imitation of the SimTest program. 
  An entry point to fast prototyping of controllers using the Python API.

klampt\_browser
---------------

``klampt_browser`` is the most convenient way to browse through many Klamp't files.  It handles
any resource type that can be visualized, and multiple items (including folders) can be
examined easily using a split-screen visualization.

When you launch the program with no arguments, you will be greeted with a file browser
and empty scene:

.. image:: _static/images/klampt_browser1.png

If you then navigate the file tree to some Klampt items, like worlds, you can select them
and they will pop up in the scene.  Multiple items can be selected using Ctrl + clicking.

.. image:: _static/images/klampt_browser2.png

The browser has a "reference world" that you will need to add elements to.  By default, it
is empty. You can configure the reference world on the command line, such as::

    klampt_browser Klampt-examples/data/athlete_plane.xml

Or, you may select the elements you want in your world, like world XML files, robots,
rigid objects, and static meshes, and click the "Add to World" button.

Once you have a reference world, selecting resources like Config,
Configs, Trajectory, and IKGoal will be displayed in context of the first
robot in the reference world.  For example, now that the athlete\_plane.xml world
has been set up as the reference world, selecting the
``Klampt-examples/data/motions/athlete_flex.path`` file will show an animation:

.. image:: _static/images/klampt_browser3.png


klampt\_control
---------------

``klampt_control`` allows you to operate robots in real time, to debug implementations
of Robot Interface Layer (RIL) controllers, and launch RIL controllers in server or client
mode.

A controller script is a Python file or module that contains a function
``make(RobotModel) -> RobotInterfaceBase``.  We recommend including such a script
in a :class:`~klampt.model.robotinfo.RobotInfo` JSON file, which specifies
the controller, model, parts, and end effectors::

   klampt_control Klampt-examples/robotinfo/ur5_sim.py

.. image:: _static/images/klampt_control.png

Alternatively, the script may be specified directly on
the command line along with the associated robot or world model.  The following
launches an interface to control a physical UR5 robot::

   klampt_control Klampt-examples/robotinfo/controllers/ur5/ur5_ril.py Klampt-examples/data/robots/ur5.rob

The default kinematic simulation interface is specified with ``klampt.control.simrobotcontroller``,
so the following controls a virtual UR5::

   klampt_control klampt.control.simrobotcontroller Klampt-examples/data/robots/ur5.rob

Similarly, you can just include the ``--sim`` flag::

   klampt_control --sim Klampt-examples/data/robots/ur5.rob

To launch a controller as an XML-RPC server, you can simply pass the ``--server`` flag::

   klampt_control --server 0.0.0.0:7881 Klampt-examples/robotinfo/ur5_sim.rob

To run the ``klampt_control`` GUI in client mode, run::

   klampt_control --client http://localhost:7881 

If you are not running on the same machine or do not have the same directory structure, you
will need to specify the robot file as well::

   klampt_control --client http://[SERVER_IP]:7881 Klampt-examples/robotinfo/ur5_sim.rob



klampt\_path
------------

``klampt_path`` performs basic editing of Trajectory (``.path``), Configs (``.configs``),
and MultiPath (``.xml``) files.  You can shift / scale the time, and concatenate multiple
paths together.

klampt\_sim
-----------

``klampt_sim`` performs simulation of a world, and can prototype Python controllers specified
in the experimental Controller API.  The following image shows the output from::

    klampt_sim Klampt-examples/data/hubo_plane.xml

.. image:: _static/images/klampt_sim.png

klampt\_resource
-------------------

``klampt_resource --robot=INPUT_ROBOT --transfer=TARGET_ROBOT INPUT_FOLDER [OUTPUT_FOLDER]`` 
converts all compatible resources from one robot to another.  The robots must share link names.
Applicable resource types are:
``Config (.config)``, ``Configs (.configs)``, ``Trajectory (.path, .traj)``, ``MultiPath (.xml)``,
``IKObjective (.ikgoal)``, ``Hold (.hold)``, ``Grasp (.grasp)``.

``klampt_resource --thumbnails --world=WORLD INPUT_FOLDER [OUTPUT_FOLDER]`` generates a folder of 
thumbnail PNGs given a folder containing Klampt resources.  This is useful when you have 
programmatically generated many  worlds, configurations, or motions. 

``klampt_resource --convert=TYPE INPUT [OUTPUT]`` converts a resource from one type to another.
Currently only supports TYPE "json".  See ``klampt_path`` for conversions of path / trajectory
types.

Example files
-------------


The `Klampt-examples Github project <https://github.com/krishauser/Klampt-examples>`_
is a companion to the main Klampt project, and you are highly recommended to download
this to get started.

World files for different robots and problem setups are available in the
Klampt-examples/data subdirectory:

-  ``hubo*.xml``: the KAIST Hubo humanoid.
-  ``puma*.xml``: the Puma 760 industrial robot.
-  ``tx90*.xml``: the Staubli TX90L industrial robot.
-  ``baxter*.xml``: the Rethink Robotics Baxter robot.

Other test robots, objects, and environments are available in the
``Klampt-examples/data/{robots,objects,terrains}`` subdirectories. Some files of
interest may include:

-  athlete.rob: the NASA ATHLETE hexapod (incomplete, missing wheel
   geometry).
-  atlas.rob: the Boston Dynamics ATLAS robot.
-  cartpole.rob: a cart-pole balancing control problem.
-  footed\_2d\_biped.rob: a simple 2D biped mimicking a human's forward
   motion.
-  footed\_2d\_monoped.rob: a simple 2D monoped.
-  hrp2.rob: the AIST HRP-2 humanoid
-  pr2.rob: the Willow Garage PR2 robot (requires KrisLibrary to be
   built with Assimp support)
-  robonaut2.rob: the NASA Robonaut2 humanoid torso.
-  robotiQ\_3finger.rob: the RobotiQ 3-finger Adaptive Gripper.
-  simple\_2d\_biped.rob: a simple 2D biped mimicking a human's lateral
   motion.
-  swingup.rob: a simple pendulum swingup control problem.
-  plane.env: a flat plane environment
-  block.obj: a 40cm block
-  block\_small.obj: an 8cm block


Utilities and Demos
~~~~~~~~~~~~~~~~~~~

The ``Klampt-examples/Python3/utils`` and
``Klampt-examples/Python3/demos`` folders contain a few example
utilities and programs that can be built upon to start getting a flavor
of programming Klamp't applications in Python.

Demos:

-  ``exercise_joints.py``: moves between all of a robot's joint
   extrema. Useful for debugging robot models.
-  ``gl_vis.py``: a simple visualization of a simulation using a
   visualization plugin.
-  ``gl_vis_widgets.py``: a plugin with widgets for visual editing,
   and demonstrating custom GUI menu actions.
-  ``kbdrive.py``: drive a simulated or real robot around using the keyboard.
   The first 10 joints can be driven via a positive velocity with the
   top row of keys 1,2,...,0 and a negative velocity with the second row
   of keys q,w,...,p.
-  ``mouse_capture.py``: shows how to capture mouse clicks in the
   visualizer.
-  ``path_test.py``: tests the :meth:`~klampt.model.trajectory.path_to_trajectory`
   function with various options.
-  ``planning_test.py``: performs tests of the motion planning module,
   with various options.
-  ``pose.py``: utility for visual posing of a simulated or real robot.
-  ``resource_demo.py``: demonstrates various functions of the
   `klampt.io.resource <Manual-Resources.html>`__ module.
-  ``robotiq.py``: modeling and simulating the RobotiQ 3-finger
   Adaptive Gripper. This code emulates the underactuated transmission
   mechanism of each finger.
-  ``robotiqtest.py``: performs a simulation of the RobotiQ gripper
   closing and opening on an object.
-  ``sensor_test.py``: demonstrates how to use a simulated camera sensor.
-  ``sphero.py``: simulates the Sphero 2.0 robot driving around.
-  ``trajectory_test.py``: demonstrates the various types of
   :class:`~klampt.model.trajectory.Trajectory` and the
   :meth:`~klampt.model.trajectory.execute_trajectory` function.
-  ``vis_template.py``: demonstrates several functions of the vis
   module.
-  ``workspace_test.py``: demonstrates usage of the workspace calculation
   utilities.

