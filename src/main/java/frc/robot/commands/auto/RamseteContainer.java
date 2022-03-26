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

/** Add your docs here. */
public class RamseteContainer {
    private RamseteCommand ramsete;
    private DrivetrainSubsystem m_drive;
    private Trajectory m_trajectory;

    public RamseteContainer(DrivetrainSubsystem drive, TBase trajectoryBase){
        m_drive = drive;
        m_trajectory = trajectoryBase.getTrajectory();

        ramsete = new RamseteCommand(m_trajectory, m_drive::getPose, new RamseteController(Vars.RAMSETE_B, Vars.RAMSETE_ZETA), new SimpleMotorFeedforward(Vars.DRIVE_KS, Vars.DRIVE_KV, Vars.DRIVE_KA), Vars.KINEMATICS, m_drive::getWheelSpeeds, new PIDController(Vars.AUTO_PATH_KP, 0, 0), new PIDController(Vars.AUTO_PATH_KP, 0, 0), m_drive::tankdriveVoltage, m_drive);
    }
        
  /**
   * Gets the Ramsete Command
   * @return the ramsete command to follow the path
   */
  public CommandBase getCommand() {
    return new InstantCommand(m_drive::resetOdometry, m_drive).andThen(ramsete);
  }

  /**
   * Gets the Ramsete Command
   * @return the command (and stops the drive after it finishes)
   */
  public CommandBase getCommandAndStop() {
    return new InstantCommand(m_drive::resetOdometry, m_drive)
            .andThen(ramsete)
            .andThen(new InstantCommand(m_drive::stop, m_drive));
  }
    }

