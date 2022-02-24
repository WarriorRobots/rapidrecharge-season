// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPosition extends CommandBase {
  /** Creates a new ArmPosition. */
  private ArmSubsystem m_arm;
  private double m_degrees;

  public ArmPosition(ArmSubsystem arm, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.  
    m_arm = arm;
    m_degrees = degrees;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setAngleBounded(m_degrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO change tolerance
    return Math.abs(m_arm.getPosition() - m_degrees) < Vars.ARM_TOLERANCE;
  }
}
