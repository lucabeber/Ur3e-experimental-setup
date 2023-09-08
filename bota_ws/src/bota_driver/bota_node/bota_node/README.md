# bota_node (a.k.a. minimal nodewrap)

## Overview

Implements several convenience classes and functions.

### Differences to ROS Nodes

The bota_node is a wrapper for the official [ROS node](http://wiki.ros.org/Nodes). It adds workers (high-precision
version of ros::Rate class), at the cost of less stable API. If these features are not explicitly required, it is
recommended to use the official ROS node.

### Node.hpp

Provides an interface base class bota_node::Node, which declares init, cleanup and update functions and has a
bota_worker::WorkerManager instance. Classes derived from this are compatible with the Nodewrap template. Additionally,
it forwards calls of subscribe, advertise, param, advertiseService serviceClient and addWorker calls to the above
mentioned functions.

### Nodewrap.hpp

Convencience template, designed to be used with classes derived from bota_node::Node. It automatically sets up ros
nodehandlers (with private namespace) and spinners, signal handlers (like SIGINT, ...) and calls the init function on
startup and cleanup on shutdown of the given Node.

