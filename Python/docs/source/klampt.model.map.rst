klampt.model.map module
-------------------------------------------

Defines a convenient object-oriented interface for accessing worlds, robots, objects, links, etc. For example, you can write

.. code:: python

   wm = map.map(world)
   wm.robots[0].links[4].transform

instead of

::

   world.robot(0).link(4).getTransform().

Most notably used in the `klampt.sim.batch <klampt.sim.batch.html>`__ module.



.. automodule:: klampt.model.map
    :members:
    :undoc-members:
    :show-inheritance:

