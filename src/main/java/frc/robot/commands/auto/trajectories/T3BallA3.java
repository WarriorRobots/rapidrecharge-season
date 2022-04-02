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
 * Trajectory to go from Blue2 through a waypoint then moving to Blue2
 */
public class T3BallA3 extends TBase {

  @Override
  void build() {
    start = Vars.AUTOBALLA2;
    Waypoints.add(Vars.AUTOBALL2W1);
    end = Vars.AUTOBALLA3;
  }
}
