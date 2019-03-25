.. nn_class_docs documentation master file, created by
   sphinx-quickstart on Fri Aug 17 17:05:47 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

#########################
Some insights in reinforcement learning for mobile robots
#########################

Introduction
============

RL can incredibly difficult to set up even in simulation and needless to say, even more so on real
hardware. This is a curated lost of reinforcement learning experiments in mobile robotics tested first
in a gazebo based simulation environment that I developed myself and then verified on a commercially
available mobile robot platform. I have also created a curated list of interesting experiments regarding
RL in mobile robots which will surely be beneficial to the reader for reference.

Why it matters
============

1. A minimalist and intuitive API similar to pytorch and keras
2. Doesn't confuse the beginner by providing a long list of hyperparameter choices
3. Compare perfomace of different optimizers easily
4. Plot the hidden layers as weights effortlessly to visualize how image reocognition models working
5. Option to display the math to get a idea regarding the theory applied


Contents
========

.. toctree::
   :maxdepth: 2
   :caption: Getting started

   Why it exsists <getting_started>

.. toctree::
   :maxdepth: 2
   :caption: Neural Networks

   nn_intro

.. toctree::
   :maxdepth: 2
   :caption: Initialization

   Initialization <Initialization>

.. toctree::
   :maxdepth: 2
   :caption: Forward propogation

   About Forward prop<forward_prop>

.. toctree::
   :maxdepth: 2
   :caption: Loss Functions

   loss_functions/index

.. toctree::
   :maxdepth: 2
   :caption: Optimizers

   About Optimizers <optimizers>
   Gradient descent
   Momentum
   RMSprop
   Adam
