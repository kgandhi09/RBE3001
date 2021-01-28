# Introduction

This is the final project of RBE 3001 course. The aim of this project is to build an automated robotic sorting system. The robot has to detect and localize the objects within its workspace using machine vision, pick them up by controlling the robotic arm's end effector tool, classify them based on appearance and release them within a predefined area. The team learned about various topics of robotic manipulation using a provided 3 degrees of freedom robotic manipulator. Furthermore, calculated the forward, inverse and differential kinematics of the robot for use in trajectory generation. Lastly, team implemented machine vision algorithm to sort objects by color and disk size.

## Topics covered:
- System Architecture and Joint Level Control
- Forward Kinematics and Motion Planning
- Inverse Kinematics and Task-Space Motion Planning
- Differential Kinematics
- Computer Vision

## Forward Kinematics
In order to calculate the forward kinematics for the arm, team used the Denavit-Hartenverg convention and created a table of DH parameters based on the kinematics structure of the arm.

## Inverse Kinematics
Given a tip position p(x), p(y), p(z), team calculated the series of joint angles in radians to be sent to each of the robot's joint, which was done through inkin() function. The team used a geometric approach to derive the equations drawing out values of these joint angles.

## Trajectory Planning
Team used two approaches for trajectory generation: Cubic polynomial technique and Quintic Polynomial technique. For the final project, the team decided to use cubic
trajectories. While quintic trajectory planning offered many advantages, the robot’s movement was not as smooth as anticipated.

## Jacobian & Differential Kinematics
jacob0 function was created to calculate the Jacobian matrix, fwvelkin function was created in order to calculate forward velocity kinematics of the robotic arms.

## Force-Torque Relationship
In order to calculate the Dynamic Force-Torque relationship, students applied energy-based techniques using Lagrangian method. 

## Computer Vision
Team used the computer vision toolkit in MatLab to calibrate the camera using the calibration grid, detect the objects, perform miscellanous masking to avoid detection of objects outside the workspace, and convert & communicate the position of object to the robotic arm.

## Link to the Demo:
https://www.youtube.com/watch?v=ANdkAOYKjz4e

## Link to the final Report:
If interested in reading how different strategies were carried out, here's the link to the report that explains everything in detail: https://drive.google.com/file/d/1NQZ4J4zxUyeTlzhr2wb4W1P6sHYZuBAu/view?usp=sharing
