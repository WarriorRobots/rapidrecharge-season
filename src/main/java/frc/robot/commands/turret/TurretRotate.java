// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretRotate extends CommandBase {
  /** Creates a new TurretRotate. */
  private TurretSubsystem m_turret;
  private DoubleSupplier m_input;
  public TurretRotate(TurretSubsystem turret, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_input = input;
    addRequirements(this.m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.rotateSafety(m_input.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
