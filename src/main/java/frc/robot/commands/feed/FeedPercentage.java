// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feed;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeedSubsystem;

public class FeedPercentage extends CommandBase {
  /** Creates a new FeedPercentage. */
  public FeedPercentage(FeedSubsystem feed, DoubleSupplier percent) {
    // TODO
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO give the feed the value of the supplier
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO stop the feed
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
