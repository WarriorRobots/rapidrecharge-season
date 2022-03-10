// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbMagic extends CommandBase {
  private ClimbSubsystem m_climb;
  private double m_inches;

  /**
   * Creates a new ClimbMagic.
   * 
   * @param climb
   * @param inches Inches of climb travel (0 is when climb has not yet moved)
   */
  public ClimbMagic(ClimbSubsystem climb, double inches) {
    m_climb = climb;
    m_inches = inches;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.ClimbMagic(m_inches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // XXX this is fine since the climb is not sequenced
  }
}
