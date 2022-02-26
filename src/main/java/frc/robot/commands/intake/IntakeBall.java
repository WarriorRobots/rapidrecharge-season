// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBall extends CommandBase {
  /** Creates a new IntakeBall. */
  IntakeSubsystem m_IntakeSubsystem;
  FeedSubsystem m_feedSubsystem;
  public IntakeBall(IntakeSubsystem intake, FeedSubsystem feed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intake;
    addRequirements(m_IntakeSubsystem);
    m_feedSubsystem = feed;
    addRequirements(m_feedSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.setPercentage(Vars.INTAKE_PERCENT, Vars.INTAKE_PERCENT);
    if(!m_feedSubsystem.containsBall()){
      m_IntakeSubsystem.setPercentage(Vars.INTAKE_PERCENT, Vars.INTAKE_PERCENT);
    }else{
      m_feedSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
    m_feedSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
