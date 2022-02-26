// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFeed extends CommandBase {
  /** Creates a new ShooterFeed. */
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  FeedSubsystem m_feed;
  public ShooterFeed(ShooterSubsystem shooter, IntakeSubsystem intake, FeedSubsystem feed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    addRequirements(m_shooter);
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
   //XXX todo now
   // if(Math.abs(m_shooter.getRPMFront()-m_shooter.get))
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
