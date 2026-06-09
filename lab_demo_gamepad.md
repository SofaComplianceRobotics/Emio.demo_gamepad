# Lab Demo Gamepad


<!-- Highlight the summary / overview of the lab -->
::: highlight
##### Overview
This demo shows how to use the `Sofa.GamepadController` module to control a SOFA simulation using a gamepad to move the gripper of Emio 
It uses the `inputs` Python package to handle gamepad events. Press the `A`button to toggle the precision mode. Left `Z` trigger closes the gripper and right `Z` trigger opens it. You can also click the `B` button to invert the vertical axis of the sticks.
:::


## Install the Python Packages
This lab uses [Sofa.GamepadController](https://github.com/SofaComplianceRobotics/Sofa.GamepadController) project, which relies on the `inputs` Python package to handle gamepad events. You can install it with pip.

Click the button to install:

#python-button(pyargs=["-m", "pip", "install", "--target", "assets/labs/demo_gamepad/modules/site-packages", "git+https://github.com/SofaComplianceRobotics/Sofa.GamepadController.git@main"])

## Run the demo

The demo moves the effector target of Emio using the left and right sticks of the gamepad. 
Press the `A`button to toggle the precision mode.
Left `Z` trigger closes the gripper and right `Z` trigger opens it.
You can also click the `B` button to invert the vertical axis of the sticks.

#runsofa-button(file="assets/labs/demo_gamepad/lab_demo_gamepad.py")

#include(assets/labs/demo_gamepad/sections/authors.md)


