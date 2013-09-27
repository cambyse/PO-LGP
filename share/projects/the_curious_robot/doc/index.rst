.. The Curious Robot documentation master file, created by
   sphinx-quickstart on Thu Sep 26 16:43:47 2013.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to The Curious Robot's documentation!
=============================================

This is the implementation of "The Curious Robot" (TCR) project.

The Curious Robot (TCR) is about the challenge of gaining controllability over
external DoFs in complex environments, i.e., exploring and learning the
environment and building a model of the world with the affordances of the
objects in the world.


Task for TCR
--------------
Learn model s.t. I can predict the outcome of my actions.

Behavior must be able to set
- goal position
- push vector
- (additional actions like "grap")

- simulate action in ors
- do action in the world


How to choose actions (and how to learn)?
-----------------------------------------

- given a list of objects (shapes whatever)
- take "interesting objects"
- move towards object
- each object has a list of popential interesting properties

  - type=[Fix, dynamic]
  - joints
  - limits
  - weight
  - friction
  - etc

- explore and learn uncertain properties


Contents:
----------

.. toctree::
   :maxdepth: 2

   install
   architecture
   discussion



.. Indices and tables
.. ==================

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`

