.. nn_class_docs documentation master file, created by
   sphinx-quickstart on Fri Aug 17 17:05:47 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

#########################
Some insights in reinforcement learning for mobile robots
#########################

.. nn_class_docs documentation master file, created by
   sphinx-quickstart on Fri Aug 17 17:05:47 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

I have included a variety of environments for testing RL algorithms along with for accessing the sensors ,actuators. Some of these
environments were adapted from open sourced repositories and I do not own the right to them

Maze Brick World
============

.. image:: img/maze_robot.png
  :width: 400
  :alt: Alternative text

Kitchen and Dining World
========================

.. image:: img/kitchen_dining_world.png
  :width: 400
  :alt: Alternative text


No Obstacle World
========================

.. image:: path/noobstacle.png
  :width: 400
  :alt: Alternative text

Static Obstacle World
========================

.. image:: path/staticobstacle.jpg
  :width: 400
  :alt: Alternative text

Dynamic Obstacle World
========================

.. image:: path/dynamicobstacle.jpg
  :width: 400
  :alt: Alternative text

Launching environment with desired environment
========================

::

   roslaunch diff_wheeled_robot_gazebo diff_wheeled_robot.launch world:=kitchen_dining.world

Arguments
--------------

  - world : Any world from the above list. By default, maze_loop_brick world is launched
  - Plot loss == True : Plots how the loss varies as the network is trained
