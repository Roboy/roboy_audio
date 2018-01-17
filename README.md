# DialogSystem
[![Documentation Status](https://readthedocs.org/projects/roboy-audio/badge/?version=master)](http://roboydialog.readthedocs.io/en/master/?badge=master)

## What is it?

This repository contains the audio related skills for the humanoid robot Roboy (roboy.org).

## How does it work?

It procecess the microphone data from Matrix Creator (www.matrix.one) in two different ways. It wether reads the data directly out of the board related FPGA and processes it. Or it streams the data onto a ROS topic, from which on other modules can submit to this topic and process it further.

Most of the things (direct processing, streaming) run in C++. But there is also a template for receiving the stream in Python.

## How to run it?

Will be defined in the future.

## How to extend it?

Write cool stuff and add it to the repo.
