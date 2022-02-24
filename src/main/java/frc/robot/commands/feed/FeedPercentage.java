// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feed;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeedSubsystem;

public class FeedPercentage extends CommandBase {
  /** Creates a new FeedPercentage. */
  private FeedSubsystem m_feed;
  private DoubleSupplier m_percent;

  public FeedPercentage(FeedSubsystem feed, DoubleSupplier percent) {
    // Use addRequirements() here to declare subsystem dependencies.    
    m_feed = feed;
    m_percent = percent;
    addRequirements(m_feed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feed.setPercentage(m_percent.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
