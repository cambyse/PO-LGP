Intro to our python PR2 interface
=================================

This is our action interface to the PR2 robot. It works both in simulation and
on the real robot, although we're not yet at a point where this can be
considered stable.

Install & Dependencies
-----------------------

You must be able to compile the MLR "Actions" code::

    cd ~/git/mlr/share/src/Actions
    make -f Makefile.gof

You need the following dependencies:

- python 2.7
- numpy
- python-enum34

For using the docs install the following:

::

    pip install --user sphinx sphinx_rtd_theme enum34

Build the docs::

    cd ~/git/mlr/share/examples/Actions/swig/doc
    make html


Startup procedure
-----------------

The interface is basically a wrapper around Marc's relational machine with some
sugar for common tasks. It can be used both, within scripts and in an
interactive manner. In every case make shure ``mlr/share/lib`` is in your
``$LD_LIBRARY_PATH`` and ``mlr/share/src/Actions`` is in your ``$PYTHONPATH``.

Alternatively create symlink::

    cd ~/git/mlr/share/examples/Actions/swig
    ln -s ../../../src/Actions/_swig.so .
    ln -s ../../../src/Actions/swig.py .

For a first run make sure ``useRos = 0`` in the ``MT.cfg`` file. If you want to
drive the real robot, you have to change that to ``useRos = 1``, but for a
first try you should always run simulation first.

For the interactive mode then run::

    $ ipython -i magic.ipy

That will load the interface and start a graphical simulation of it, which
is a OpenGL rendering of the ORS scene.

Basic interface
---------------

The direct translation of the relational machine would be to write a relational
fact and add it to the logic state::

    >>> fact = "(HomingActivity){tol=.01}"
    >>> interface.setFact(fact)

This starts a ``HomingActivity`` and the robot should move to its base
position. However, you must take care to remove the fact after the activity
has converged::

    >>> interface.stopFact(fact)
    >>> interface.stopFact("(HomingActivity conv)")

Not very convenient. So instead there is ``run()`` function taking care of
that::

    >>> run(fact)

That starts the fact, wait for its convergence and removes all facts again.

You also have basic access to the underlying ors structure. See
:ref:`section-interface` for a detailed description.

Activities
----------

While this is already much better than taking care of everything on you own,
writing the facts can be a tedious task. Therefore there are several
``Activity`` subclasses, which generate facts for common tasks. See
:ref:`section-actions` for a complete list.

For the ``HomingActivity`` there is for instance a ``HomingActivity`` class,
which can be easily used::

    >>> run(HomingActivity())

Activities all have a couple of parameters, that determine the actual behavior
you see. For instance you can set the natural gains of the PD controller with::

    >>> homing = HomingActivity()
    >>> homing.natural_gains = [3, .6, 10, 11]

Complex Behaviors
-----------------

While this gives you the ability to create simple sequential behaviors,
often a more complex structure is needed.

For letting something run while something else runs there is a contextmanger::

    >>> with running(GazeAtActivity(s.endeffL)):
            run(ReachActivity(s.endeffL, s.mymarker))

This let the robot look at its end-effector while reaching a marker with it.

The ``with:`` construct doesn't look if the Activity in ``running()``
converges. Whenever the activities within the block are converged it stops. If
you want to run several activities in parallel but you want to wait for all to
converge you can give the ``run()`` function a tuple of activities::

    >>> run((ReachActivity(s.endeffL, s.mymarker), ReachActivity(s.endeffR, s.door))

Tries to reach a marker with the left end-effector and a door with the right
one.

.. _section-plan-format:

The Plan Format
---------------
Soon you will be in the need to move such complex behaviors or plans around.
We have a format to achieve that.

A plan is a lightwight datastructure to store sequential and simultaneous
activities. A single Activity is already a plan. To create more complex
plans, the following construxts are possible:

* A list of activities or plans is run sequentially. Each one must be
  converged for the next one to start
* A tuple of activities or plans is run simultaneously. Everything is
  flatten, i.e. ``(a, [b, c], d)`` is equally treated as ``(a, b, c, d)``. All
  activities have to be converged for the tuple to be considered converged
* A dict with two entries. First ``"with"`` contains a list of activities,
  second ``"plan"`` contains a plan. The list of activities in the "with" list
  are run simultaneous to the plan. However, when the last activity of the
  plan is converged they are stopped regardless of the convergence status.

For example a plan could look like::

     plan = [align_gripper_with_plane(*plane, side=side),
             {"with": [align_gripper_with_plane(*plane, side=side),
                       gaze_at(endeff)],
              "plan": [(open_gripper(side),
                        reach(shape, offset=pre_grasp_offset, with_=endeff)),
                       reach(shape, offset=grasp_offset, with_=endeff),
                       close_gripper(side),
                       MoveAlongAxisActivity(endeff, axis, distance),
                       open_gripper()]
             }]
