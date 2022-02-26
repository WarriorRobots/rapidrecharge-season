// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterPrep extends CommandBase {
  IntakeSubsystem m_intake;
  FeedSubsystem m_feed;

  /** Creates a new ShooterPrep. */
  public ShooterPrep(IntakeSubsystem intake, FeedSubsystem feed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
    m_feed = feed;
    addRequirements(m_feed);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPercentage(Vars.INTAKE_PERCENT, Vars.INTAKE_PERCENT);
    m_feed.setPercentage(Vars.INTAKE_PERCENT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_feed.containsBall();
  }
}
