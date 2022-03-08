// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Vars;
import frc.robot.commands.arm.ArmHoldPosition;
import frc.robot.commands.arm.ArmStabilize;
import frc.robot.commands.intake.IntakeBall;
import frc.robot.commands.shooter.AimShootFeed;
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
public class AutoShootForward extends SequentialCommandGroup {
  /** Creates a new AutoPickupShoot. */
  public AutoShootForward(DrivetrainSubsystem drive, ShooterSubsystem Shooter, TurretSubsystem Turret, CameraSubsystem Camera, IntakeSubsystem Intake, FeedSubsystem Feed, ArmSubsystem Arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ArmStabilize(Arm),
    new PrintCommand("going backwards!"),
    new AutoLinear(drive, Vars.AUTO_BACKUP_DISTANCE),
    new PrintCommand("going forwards!"),
    new AutoLinear(drive, Vars.AUTO_FORWARD_DISTANCE),
    new PrintCommand("shooting!"),
    new ParallelDeadlineGroup(
     new WaitCommand(Vars.AUTO_WAIT_TO_SHOOT_TIME), 
    new AimShootFeed(Shooter, Turret, Intake, Feed, Camera,()->Vars.SHOOTER_BACK_DEFAULT_RPM, ()->Vars.SHOOTER_FRONT_DEFAULT_RPM)
    ),
    new PrintCommand("finished auto!")

    

    
      
      

    );
  }
}
