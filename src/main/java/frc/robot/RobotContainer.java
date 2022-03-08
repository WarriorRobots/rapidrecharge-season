
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.arm.ArmHoldPosition;
import frc.robot.commands.arm.ArmLinear;
import frc.robot.commands.arm.ArmMakeRoom;
import frc.robot.commands.arm.ArmPosition;
import frc.robot.commands.arm.ArmStabilize;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.camera.CameraChangePipeline;
//import frc.robot.commands.drive.Linear;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.feed.FeedPercentage;
import frc.robot.commands.intake.IntakeBall;
import frc.robot.commands.intake.IntakePercentage;
import frc.robot.commands.shooter.AimShootFeed;
import frc.robot.commands.shooter.ShooterFeed;
import frc.robot.commands.shooter.ShooterPercentage;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretPreset;
import frc.robot.commands.turret.TurretRotate;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  protected static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  protected static final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();
  protected static final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
  protected static final ArmSubsystem m_ArmSubsytem = new ArmSubsystem();
  protected static final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  protected static final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  protected static final FeedSubsystem m_FeedSubsystem = new FeedSubsystem();

  // Drivetrain
  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, () -> IO.getLeftY(), () -> IO.getRightY());
  // private final Linear m_linear = new Linear(m_drivetrain);
  // private final SequentialCommandGroup m_shooterSequence = new
  // SequentialCommandGroup(
  // new ShooterPrep(m_IntakeSubsystem, m_FeedSubsystem),
  // new ShooterFeed(m_ShooterSubsystem, m_IntakeSubsystem, m_FeedSubsystem,
  // ()->FrontRPM.getDouble(0), ()->BackSpinRPMINPUT.getDouble(0)){public void
  // end(boolean interrupted){/* This is empty is to not stop the motor from
  // rev-ing*/}}
  // ){public void end(boolean
  // interrupted){m_IntakeSubsystem.stop();;m_FeedSubsystem.stop();}};

  // private final SequentialCommandGroup m_ShooterAimAndShoot = new
  // SequentialCommandGroup(
  // new TurretAim(m_CameraSubsystem, m_TurretSubsystem),
  // new ParallelCommandGroup(
  // new TurretAim(m_CameraSubsystem, m_TurretSubsystem).perpetually(),
  // new ShooterFeed(m_ShooterSubsystem, m_IntakeSubsystem, m_FeedSubsystem,
  // ()->FrontRPM.getDouble(Vars.SHOOTER_FRONT_ESTIMATED_RPM),
  // ()->BackSpinRPMINPUT.getDouble(Vars.SHOOTER_BACK_ESTIMATED_RPM)){public void
  // end(boolean interrupted){/* m_ShooterStop will be called to stop the shooter
  // */
  // m_IntakeSubsystem

  // .stop();
  // m_FeedSubsystem.stop();
  // }}
  // )
  // );

  // private final AimShootFeed m_ShooterAimAndShoot = new
  // AimShootFeed(m_ShooterSubsystem, m_TurretSubsystem, m_IntakeSubsystem,
  // m_FeedSubsystem, m_CameraSubsystem,
  // ()->FrontRPM.getDouble(Vars.SHOOTER_FRONT_ESTIMATED_RPM),
  // ()->BackSpinRPMINPUT.getDouble(Vars.SHOOTER_BACK_ESTIMATED_RPM));
  private final ShooterFeed m_ShootAndFeed = new ShooterFeed(m_ShooterSubsystem, m_IntakeSubsystem, m_FeedSubsystem,
      () -> DashboardContainer.getInstance().FrontRPMInput(), () -> DashboardContainer.getInstance().BackRPMInput());

  private final Command m_ShooterButton = new SequentialCommandGroup(
      // while making sure there are no balls touching the shooter...
      new ParallelDeadlineGroup(new WaitCommand(Vars.SHOOTER_BACK_FEED_TIME),
          new FeedPercentage(m_FeedSubsystem, Vars.FEED_REVERSED_PERCENT_SLOW)),
      // and then shoot and feed while aiming
      new AimShootFeed(m_ShooterSubsystem, m_TurretSubsystem, m_IntakeSubsystem, m_FeedSubsystem, m_CameraSubsystem,
          () -> DashboardContainer.getInstance().FrontRPMInput(), () -> DashboardContainer.getInstance().BackRPMInput()));

  private final SequentialCommandGroup m_ShooterBoostButton = new SequentialCommandGroup(
      new ParallelCommandGroup(
        // the arm should move away from the shooter...
        new ArmMakeRoom(m_ArmSubsytem),
        // while making sure there are no balls touching the shooter...
        new ParallelDeadlineGroup(new WaitCommand(Vars.SHOOTER_BACK_FEED_TIME),
            new FeedPercentage(m_FeedSubsystem, Vars.FEED_REVERSED_PERCENT_SLOW))),
      // and then shoot and feed while aiming
      new AimShootFeed(m_ShooterSubsystem, m_TurretSubsystem, m_IntakeSubsystem, m_FeedSubsystem, m_CameraSubsystem,
          () -> DashboardContainer.getInstance().FrontBoostRPMInput(), () -> DashboardContainer.getInstance().BackBoostRPMInput()));
  private final Command m_ShooterButtonLeft = new SequentialCommandGroup(
      // while making sure there are no balls touching the shooter...
      new ParallelDeadlineGroup(new WaitCommand(Vars.SHOOTER_BACK_FEED_TIME),
          new FeedPercentage(m_FeedSubsystem, Vars.FEED_REVERSED_PERCENT_SLOW)),
      // and then shoot and feed
      new ShooterFeed(m_ShooterSubsystem, m_IntakeSubsystem, m_FeedSubsystem,
          () -> DashboardContainer.getInstance().FrontRPMInput(),
          () -> DashboardContainer.getInstance().BackRPMInput()));

  private final ParallelCommandGroup m_ShooterPrep = new ParallelCommandGroup(
      new TurretAim(m_CameraSubsystem, m_TurretSubsystem).perpetually(),
      new ShooterRPM(m_ShooterSubsystem, () -> DashboardContainer.getInstance().FrontRPMInput(), () -> DashboardContainer.getInstance().BackRPMInput()) {
        public void end(boolean interrupted) {
          /* m_ShooterStop will be called to stop the shooter */}
      },
      new ShooterPrep(m_IntakeSubsystem, m_FeedSubsystem));
  // private final InstantCommand m_ShooterStop = new
  // InstantCommand(()->m_ShooterSubsystem.stop(), m_ShooterSubsystem);
  // private final SequentialCommandGroup m_Shooter

  /**
   * Clears the shooter and runs the shooter at an rpm for the shooter to then be
   * fed
   */
  // private final SequentialCommandGroup m_ShootAndFeed = new
  // SequentialCommandGroup(
  // new ShooterFeed(m_ShooterSubsystem, m_IntakeSubsystem, m_FeedSubsystem,
  // ()->FrontRPM.getDouble(0), ()->BackSpinRPMINPUT.getDouble(0)),
  // new
  // ShooterRPM(m_ShooterSubsystem,()->FrontRPM.getDouble(0),()->BackSpinRPMINPUT.getDouble(0)
  // )
  // );
  // private final ShooterFeed m_ShootAndFeed = new
  // ShooterFeed(m_ShooterSubsystem, m_IntakeSubsystem, m_FeedSubsystem,
  // ()->FrontRPM.getDouble(Vars.SHOOTER_FRONT_ESTIMATED_RPM),
  // ()->BackSpinRPMINPUT.getDouble(Vars.SHOOTER_BACK_ESTIMATED_RPM));

  // Camera
  private final CameraChangePipeline m_TrackingCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem,
      CameraSubsystem.Tracking_Pipline);
  private final CameraChangePipeline m_DriverCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem,
      CameraSubsystem.Drive_Pipline);

  // TODO robot should be able to shoot balls high using values from the dashboard; see: https://github.com/WarriorRobots/dummyRobot2022/blob/aeeb984605764853f1215d135f280ca5627459ee/src/main/java/frc/robot/RobotContainer.java#L36

  // Turret
  private final TurretRotate m_TurretRotate = new TurretRotate(m_TurretSubsystem, () -> IO.getXBoxRightX());
  private final TurretPreset m_TurretPreset90 = new TurretPreset(m_TurretSubsystem, 90);
  private final TurretPreset m_TurretPreset180 = new TurretPreset(m_TurretSubsystem, 180);
  // private final TurretPreset m_TurretPresetMinus90 = new
  // TurretPreset(m_TurretSubsystem, -90);
  private final TurretPreset m_TurretPresetMinus50 = new TurretPreset(m_TurretSubsystem, -50);
  private final TurretPreset m_TurretPreset0 = new TurretPreset(m_TurretSubsystem, 0);
  private final TurretAim m_TurretAim = new TurretAim(m_CameraSubsystem, m_TurretSubsystem);// {public boolean
                                                                                            // isFinished(){return
                                                                                            // false;}};
  // Arm
  private final ArmLinear m_ArmLinear = new ArmLinear(m_ArmSubsytem, () -> IO.getXBoxLeftY());
  private final ArmPosition m_ArmPosition0 = new ArmPosition(m_ArmSubsytem, 0);
  private final ArmPosition m_ArmPosition90 = new ArmPosition(m_ArmSubsytem, 90);
  private final ArmPosition m_ArmPositionIN = new ArmPosition(m_ArmSubsytem, Vars.ARM_IN);
  private final ArmPosition m_ArmPositionIntake = new ArmPosition(m_ArmSubsytem, Vars.ARM_ANGLE_PICKUP);
  private final ArmZero m_ArmZero = new ArmZero(m_ArmSubsytem);
  private final ArmStabilize m_ArmStabilize = new ArmStabilize(m_ArmSubsytem);

  /**
   * Runs once at the start of teleop
   * 
   * @param enable Set to true if the robot was just enabled
   *               (there are scenarios where this could be false, eg moving from
   *               auto to teleop)
   */
  public void startup(boolean enable) {

    if (enable) {
      // run the commands that only occur when the enable button was just pressed
      // WHen robot enabled arm shouldnt go anywhere; Reason for "ArmStabilize"
      new ArmStabilize(m_ArmSubsytem).schedule();
    }

    // run the commands for startup

  }

  private final ParallelCommandGroup m_IntakeSequence = new ParallelCommandGroup(
      new ArmHoldPosition(m_ArmSubsytem, Vars.ARM_ANGLE_PICKUP),
      new IntakeBall(m_IntakeSubsystem, m_FeedSubsystem, Vars.INTAKE_PERCENT, Vars.SHOOTER_SLOW_INTAKE));
  private final ParallelCommandGroup m_DriverIntakeSequence = new ParallelCommandGroup(
      new ArmHoldPosition(m_ArmSubsytem, Vars.ARM_ANGLE_PICKUP),
      new IntakeBall(m_IntakeSubsystem, m_FeedSubsystem, Vars.INTAKE_PERCENT, Vars.SHOOTER_SLOW_INTAKE));

  private final SequentialCommandGroup m_UNJAMintake = new SequentialCommandGroup(
      new ArmPosition(m_ArmSubsytem, Vars.ARM_ANGLE_PICKUP),
      new ParallelCommandGroup(
          new FeedPercentage(m_FeedSubsystem, Vars.FEED_REVERSED_PERCENT),
          new IntakePercentage(m_IntakeSubsystem, Vars.FEED_REVERSED_PERCENT, Vars.FEED_REVERSED_PERCENT)));
  // XXX Zero arm command (should move linearly to find the hall effect)

  // Write to DashBoard
  private final RunCommand m_DashWriter = new RunCommand(() -> DashboardContainer.getInstance().putDashboard()) {
    public boolean runsWhenDisabled() {
      return true;
    }
  };
  // Feed
  private final FeedPercentage m_FeedPercentage = new FeedPercentage(m_FeedSubsystem, 1.0);
  private final FeedPercentage m_FeedPercentageBack = new FeedPercentage(m_FeedSubsystem, -1.0);
  private final FeedPercentage m_ReverseFeed = new FeedPercentage(m_FeedSubsystem, Vars.FEED_REVERSE_BUTTON);
  // Intake
  private final IntakePercentage m_IntakePercentage = new IntakePercentage(m_IntakeSubsystem, Vars.INTAKE_PERCENT,
      Vars.INTAKE_PERCENT);
  private final IntakePercentage m_IntakePercentageBack = new IntakePercentage(m_IntakeSubsystem, -Vars.INTAKE_PERCENT,
      -Vars.INTAKE_PERCENT);
  private final IntakeBall m_IntakeBall = new IntakeBall(m_IntakeSubsystem, m_FeedSubsystem, Vars.INTAKE_PERCENT,
      Vars.SHOOTER_SLOW_INTAKE);
  // Shooter
  private final ShooterRPM m_ShooterRPM = new ShooterRPM(m_ShooterSubsystem, () -> DashboardContainer.getInstance().FrontRPMInput(),
      () -> DashboardContainer.getInstance().BackRPMInput());
  private final ShooterPercentage m_ShooterReverse = new ShooterPercentage(m_ShooterSubsystem,
      () -> Vars.SHOOTER_FRONT_REVERSE, () -> Vars.SHOOTER_BACK_REVERSE);

  /**
   * The container for the robot. Conj
   * tains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_tankDrive);
    CommandScheduler.getInstance().schedule(m_DashWriter);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    IO.xbox_LT.whileHeld(m_UNJAMintake).whenReleased(m_ArmPosition0);
    IO.xbox_RT.whileHeld(m_ShooterReverse);
    IO.xbox_RB.whileHeld(m_IntakeSequence).whenReleased(m_ArmPosition0);
    IO.xbox_Y.whenPressed(m_TurretPreset0);
    IO.xbox_B.whenPressed(m_TurretPreset180);
    IO.xbox_X.whenPressed(m_ArmPositionIN);
    IO.xbox_A.whenPressed(m_ArmPositionIntake);
    IO.xboxUp.whileHeld(m_ArmPosition0);
    IO.xbox_L_JOYSTICK.whileHeld(m_ArmLinear);
    IO.xbox_R_JOYSTICK.whileHeld(m_TurretRotate);
    IO.xbox_LB.whileHeld(m_ReverseFeed);

    IO.leftJoystick_1.whileHeld(m_ShooterButtonLeft);
    IO.leftJoystick_4.whileHeld(m_ShooterBoostButton);
    IO.rightJoystick_1.whileHeld(m_ShooterButton);
    IO.rightJoystick_2.whileHeld(m_DriverIntakeSequence).whenReleased(m_ArmPosition0);
    IO.rightJoystick_12.whenPressed(m_ArmZero.andThen(m_ArmPositionIN)); // arm is commanded to the IN position instead of stazilize because commanding stabilize runs into the 3d printed blocks
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new CommandBase() {

    };
  }

  /**
   * Stops all the devices on the robot.
   * Used when the robot disables to not allow any processes to keep running.
   * (E.G. if the auto is stopped and the shooter is not told to stop, it is told
   * to stop here.)
   * 
   * @return
   */
  public Command getStopAll() {
    return new InstantCommand(() -> {
      m_drivetrain.stop();
      m_ShooterSubsystem.stop();
      m_ShooterSubsystem.stop();
      m_FeedSubsystem.stop();
      m_ArmSubsytem.stop();
      m_IntakeSubsystem.stop();
      // m_climb.stopWinch();
    }) {
      public boolean runsWhenDisabled() {
        return true;
      }
    };
  }
}
