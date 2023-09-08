# BOTA Node

## Overview

Set of wrapper packages to handle multi-threaded ROS nodes.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

Please report bugs and request features using the [Issue Tracker](https://gitlab.com/botasys/bota_driver/-/issues).

## Packages

This is only an overview. For more detailed documentation, please check the packages individually.

### bota_node

ROS node wrapper with some convenience functions using *bota_worker*.

### bota_worker

High resolution and threaded version of the ROS rate class.

### bota_signal_handler

Contains a static signal handling helper class.

