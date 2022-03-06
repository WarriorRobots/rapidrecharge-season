// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;

public class PixyHuntBall extends CommandBase {
  private DrivetrainSubsystem m_drive;
  private PixyCamSubsystem m_pixy;
  private PIDController m_pidAngle;
  private PIDController m_pidForward;
  /** Creates a new PixyHuntBall. */
  public PixyHuntBall(DrivetrainSubsystem drivetrain, PixyCamSubsystem pixy) {
    m_drive = drivetrain;
    m_pixy = pixy;
    m_pidAngle = new PIDController(Vars.PIXY_PID_ANGLE_KP, Vars.PIXY_PID_ANGLE_KI, Vars.PIXY_PID_ANGLE_KD);
    m_pidForward = new PIDController(Vars.PIXY_PID_FORWARD_KP, Vars.PIXY_PID_FORWARD_KI, Vars.PIXY_PID_FORWARD_KD);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_pixy.ballVisible()){
    double angle = m_pidAngle.calculate(m_pixy.getAngleX());
    double forwards = m_pidForward.calculate(m_pixy.getDistance());
    m_drive.arcadedriveRaw(forwards, angle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
