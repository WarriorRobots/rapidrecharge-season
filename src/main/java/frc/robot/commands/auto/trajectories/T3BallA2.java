/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Vars;

/**
 * Trajectory to backup from Blue1, and move to Blue2
 */
public class T3BallA2 extends TBase {

  @Override
  public boolean isReversed() {
      return true;
  }

  @Override
  void build() {
    start = Vars.AUTOBALLA1;
    end = Vars.AUTOBALLA2;
  }
}
