// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ClimbSubsystem;


public class ClimbPID extends CommandBase {
  private ClimbSubsystem m_climb;
  private double m_inches;
  /** Creates a new Climb. */
  /**
   * 
   * @param climb 
   * @param inches Inches of climb travel (0 is when climb has not yet moved)
   */
  public ClimbPID(ClimbSubsystem climb, double inches) {
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
    m_climb.ClimbPID(m_inches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_climb.getPosition() - m_inches) < Vars.CLIMB_TOLERANCE;
  }
}
