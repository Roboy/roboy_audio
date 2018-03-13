.. Architecture Documentation Wiki documentation master file, created by
   sphinx-quickstart on Fri Jul 29 19:44:29 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Roboy Audio
===========================================================

This project aims to implement human-like hearing skills. This with the use of the eight Microphones from the Matrix Creator board.

Status
------

Stable functionality:

- Audio stream from the 8 Mics through ROS
- Recieving Audio stream in python
- Azimutal angle localization of one Audio source, with low resolution
- Publishing the angles onto a rostopic

In development:

- Recognition of several xylophone key tones
- Localization with higher resolution


Contents:
----------

.. _usage:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Usage and Installation

  Usage/*

  .. _ScopeContext:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Interfaces and Scope

  ScopeContext/*
  
.. toctree::
   :maxdepth: 1

   about-arc42
