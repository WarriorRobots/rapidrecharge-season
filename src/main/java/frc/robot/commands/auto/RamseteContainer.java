// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Vars;
import frc.robot.commands.auto.trajectories.TBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A file to contain the Ramsete command to pass it into autonomous drive commands.
 */
public class RamseteContainer {

  private RamseteCommand ramsete;

  private DrivetrainSubsystem m_drive;
  private Trajectory m_trajectory;

  public RamseteContainer(DrivetrainSubsystem drive, TBase trajectoryBase) {
    m_drive = drive;
    m_trajectory = trajectoryBase.getTrajectory();
    // Make the trajectory to be followed relative to the robot where the robot begins at the origin.
    // This is because we will reset the robot's pose to the origin before following a path.
    // While this can lead to errors that build up after the robot fails to complete the path percisely,
    // this will garuntee that the behavior is consistant for each the Ramsete Container's command is used.
    m_trajectory = m_trajectory.relativeTo(m_trajectory.getInitialPose());

    // SmartDashboard.putNumber("Trajectory est.", m_trajectory.getTotalTimeSeconds());

    ramsete = new RamseteCommand(
      m_trajectory,
      m_drive::getPose,
      new RamseteController(Vars.RAMSETE_B, Vars.RAMSETE_ZETA),
      new SimpleMotorFeedforward(Vars.DRIVE_KS,
                                 Vars.DRIVE_KV,
                                 Vars.DRIVE_KA),
      Vars.KINEMATICS,
      m_drive::getWheelSpeeds,
      new PIDController(Vars.AUTO_PATH_KP, 0, 0),
      new PIDController(Vars.AUTO_PATH_KP, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drive::tankdriveVoltage,
      m_drive
    );
  }

  /**
   * Gets the Ramsete Command
   * @return the ramsete command to follow the path
   */
  public CommandBase getCommand() {
    // resetOdometry is required as the trajectory starts from the origin, see above
    return new InstantCommand(m_drive::resetOdometry, m_drive).andThen(ramsete);
  }

  /**
   * Gets the Ramsete Command
   * @return the command (and stops the drive after it finishes)
   */
  public CommandBase getCommandAndStop() {
    // resetOdometry is required as the trajectory starts from the origin, see above
    return new InstantCommand(m_drive::resetOdometry, m_drive)
            .andThen(ramsete)
            .andThen(new InstantCommand(m_drive::stop, m_drive));
  }
}
