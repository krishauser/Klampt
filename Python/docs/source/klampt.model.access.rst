klampt.model.access module
-------------------------------------------

Defines a convenient object-oriented interface for accessing worlds, robots, objects, links, etc. For example, you can write

.. code:: python

   from klampt.model import access
   wm = access.map(world)
   wm.robots[0].links[4].transform

instead of

::

   world.robot(0).link(4).getTransform().

Most notably used in the `klampt.sim.batch <klampt.sim.batch.html>`__ module.



.. automodule:: klampt.model.access
    :members:
    :undoc-members:
    :show-inheritance:

