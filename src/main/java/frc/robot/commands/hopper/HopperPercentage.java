// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperPercentage extends CommandBase {
  /** Creates a new HopperPercentage. */
  private HopperSubsystem m_hopper;
  private DoubleSupplier m_belt_top, m_belt_bottom;

  public HopperPercentage(HopperSubsystem hopper, DoubleSupplier top, DoubleSupplier bottom) {
    m_hopper = hopper;
    m_belt_top = top;
    m_belt_bottom = bottom;
    addRequirements(this.m_hopper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopper.setPercentage(m_belt_top.getAsDouble(), m_belt_bottom.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
