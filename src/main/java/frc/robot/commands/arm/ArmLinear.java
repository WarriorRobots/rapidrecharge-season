// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmLinear extends CommandBase {
  /** Creates a new ArmLinear. */
  private ArmSubsystem m_arm;
  private DoubleSupplier m_percentage;

  public ArmLinear(ArmSubsystem arm, DoubleSupplier percentage) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_percentage = percentage;
    addRequirements(this.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setPercentage(m_percentage.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setAngleUnbounded(m_arm.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
