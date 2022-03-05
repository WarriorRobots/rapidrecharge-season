// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterSequence extends SequentialCommandGroup {
  /** Creates a new ShooterSequence. */
  public ShooterSequence(ShooterSubsystem shooter, IntakeSubsystem intake, FeedSubsystem feed, DoubleSupplier FrontRPM, DoubleSupplier BackRPM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterPrep(intake, feed),
      new ShooterFeed(shooter, intake, feed, FrontRPM, BackRPM)

    );
  }
}
