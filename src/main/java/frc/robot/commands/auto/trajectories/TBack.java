// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * A straight path, but backwards.
 * Start at the origin and move 5 feet backwards in the x direction.
 */
public class TBack extends TBase {

  @Override
  public boolean isReversed() {
    return true;
  }

  @Override
  void build() {
    start = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
    end = new Pose2d(Units.feetToMeters(-0.83), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
  }

}