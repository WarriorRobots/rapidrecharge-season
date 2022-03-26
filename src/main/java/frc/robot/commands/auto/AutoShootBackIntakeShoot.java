// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DashboardContainer;
import frc.robot.DashboardContainer;
import frc.robot.Vars;
import frc.robot.commands.arm.ArmHoldPosition;
import frc.robot.commands.arm.ArmPosition;
import frc.robot.commands.arm.ArmStabilize;
import frc.robot.commands.auto.trajectories.TLine;
import frc.robot.commands.intake.IntakeBall;
import frc.robot.commands.shooter.AimShootFeed;
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
public class AutoShootBackIntakeShoot extends SequentialCommandGroup {
    /** Creates a new AutoBackIntake. */
    public AutoShootBackIntakeShoot(DrivetrainSubsystem drive, ShooterSubsystem Shooter, TurretSubsystem Turret,
            CameraSubsystem Camera, IntakeSubsystem Intake, FeedSubsystem Feed, ArmSubsystem Arm) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ArmStabilize(Arm),
                new PrintCommand("Shooting now"),
                new TurretPreset(Turret, Vars.TURRET_180),
                new ParallelDeadlineGroup(
                        new WaitCommand(Vars.AUTO_WAIT_TO_SHOOT_TIME),
                        new AimShootFeed(Shooter, Turret, Intake, Feed, Camera,
                                () -> Vars.AUTO_FRONT_SHOOTER_RPM,
                                () -> Vars.AUTO_BACK_SHOOTER_RPM)),
                new PrintCommand("Going Backwards!"),
                new ParallelDeadlineGroup(
                        new RamseteContainer(drive, new TLine(){public double getLengthIn() {return Vars.AUTO_INTAKE_BALL_FORWARD_DISTANCE;}}).getCommandAndStop(),
                        new ArmHoldPosition(Arm, Vars.ARM_ANGLE_PICKUP),
                        new IntakeBall(Intake, Feed, Vars.INTAKE_PERCENT, Vars.SHOOTER_SLOW_INTAKE)),
                new PrintCommand("Picked up ball! (maybe?)"),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new RamseteContainer(drive, new TLine(){public double getLengthIn() {return Vars.AUTO_INTAKE_BALL_BACKWARD_DISTANCE;}}).getCommandAndStop(),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(Vars.AUTO_WAIT_TO_SHOOT_TIME),
                                        new AimShootFeed(Shooter, Turret, Intake, Feed, Camera,
                                                () -> Vars.AUTO_FRONT_SHOOTER_RPM,
                                                () -> Vars.AUTO_BACK_SHOOTER_RPM))),
                        new ArmPosition(Arm, Vars.ARM_IN)),

                new PrintCommand("Auto Done")

        );
    }
}
