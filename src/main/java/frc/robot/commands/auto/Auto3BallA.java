// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DashboardContainer;
import frc.robot.Vars;
import frc.robot.commands.arm.ArmHoldPosition;
import frc.robot.commands.arm.ArmPosition;
import frc.robot.commands.arm.ArmStabilize;
import frc.robot.commands.auto.trajectories.T3BallA1;
import frc.robot.commands.auto.trajectories.T3BallA2;
import frc.robot.commands.auto.trajectories.T3BallA3;
import frc.robot.commands.auto.trajectories.TLine;
import frc.robot.commands.feed.FeedPercentage;
import frc.robot.commands.intake.IntakeBall;
import frc.robot.commands.intake.IntakePercentage;
import frc.robot.commands.shooter.AimShootFeed;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.turret.TurretPreset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto3BallA extends SequentialCommandGroup {
  /** Creates a new Auto3BallA. 
   * @param DashboardContainer */
  public Auto3BallA(DrivetrainSubsystem drive, ShooterSubsystem Shooter, TurretSubsystem Turret,
      CameraSubsystem Camera, IntakeSubsystem Intake, FeedSubsystem Feed, ArmSubsystem Arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmStabilize(Arm),
        new ParallelDeadlineGroup(
          new ParallelCommandGroup(
              new TurretPreset(Turret, Vars.TURRET_180),
              new RamseteContainer(drive, new T3BallA1(){}).getCommandAndStop()),
              new ArmPosition(Arm, Vars.ARM_ANGLE_PICKUP),
              new IntakeBall(Intake, Feed,Vars.INTAKE_PERCENT, 0)
          ),
          new ParallelDeadlineGroup( new WaitCommand(Vars.AUTO_SHOOTER_BACK_FEED_TIME), 
            new FeedPercentage(Feed, Vars.FEED_REVERSED_PERCENT),
            new IntakePercentage(Intake, Vars.AUTO_INTAKE_TOP_REVERSED_PERCENT, Vars.AUTO_INTAKE_BOTTOM_REVERSED_PERCENT)
          ),
          new ParallelDeadlineGroup(new WaitCommand(Vars.AUTO_WAIT_TO_SHOOT_TIME), 
          new AimShootFeed(Shooter, Turret, Intake, Feed, Camera, ()->Vars.AUTO_A1_FRONT_RPM, ()->DashboardContainer.getInstance().BackRPMInput())
          ),
          new RamseteContainer(drive, new T3BallA2(){}).getCommandAndStop(),
          new ParallelDeadlineGroup(
            new ParallelCommandGroup(
              new RamseteContainer(drive, new T3BallA3(){}).getCommandAndStop()),
              new ArmPosition(Arm, Vars.ARM_ANGLE_PICKUP),
              new IntakeBall(Intake, Feed,Vars.INTAKE_PERCENT, 0)
            ),
            new ParallelDeadlineGroup( new WaitCommand(Vars.AUTO_SHOOTER_BACK_FEED_TIME), 
            new FeedPercentage(Feed, Vars.FEED_REVERSED_PERCENT),
            new IntakePercentage(Intake, Vars.AUTO_INTAKE_TOP_REVERSED_PERCENT, Vars.AUTO_INTAKE_BOTTOM_REVERSED_PERCENT)
          ),
          new ParallelDeadlineGroup(new WaitCommand(Vars.AUTO_WAIT_TO_SHOOT_TIME), 
          new AimShootFeed(Shooter, Turret, Intake, Feed, Camera, ()->Vars.AUTO_A1_FRONT_RPM, ()->DashboardContainer.getInstance().BackRPMInput())
          )
          


      );
  }
}
