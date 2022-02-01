// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.trajectories;

/** Add your docs here. */
public abstract class TBase {

 /** Max speed of trajectory in/s*/
 final double MAX_SPEED = 0; //maxSpeed();
 /** Max acceleration of trajectory (in in/s^2) */
 final double MAX_ACCELERATION =  0; //maxAcceleration();
 /** Starting velocity of the robot along the trajectory in in/s */
 final double START_VELOCITY = 0; //startSpeed();
 /** Ending velocity of the robot along the trajectory in in/s */
 final double END_VELOCITY = 0; //endSpeed();
 /** Whether the robot drives backwards along the path */
 final boolean REVERSED = 0; // isReversed();

 /** Whether the trajectory is the left or right mirror of the trajectory. */
 final boolean LEFT;

}
