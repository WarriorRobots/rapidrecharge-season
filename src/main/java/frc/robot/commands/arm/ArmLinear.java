// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmLinear extends CommandBase {
  /** Creates a new ArmLinear. */
  public ArmLinear(ArmSubsystem arm, DoubleSupplier percentage) {
    // TODO
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO give the arm the value of the supplier
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO rotate the arm to wherever it is when this method is called (use setAngleUnbounded())
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
