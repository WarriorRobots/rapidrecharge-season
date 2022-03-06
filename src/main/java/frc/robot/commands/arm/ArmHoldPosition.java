// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHoldPosition extends CommandBase {
  /** Creates a new ArmHoldPosition. */
  ArmSubsystem m_Arm;
  double m_ArmPosition;
  public ArmHoldPosition(ArmSubsystem arm, double position) {
    addRequirements(arm);
    m_Arm = arm;
    m_ArmPosition = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setAngleBounded(m_ArmPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.toDegrees(Vars.ARM_IN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
