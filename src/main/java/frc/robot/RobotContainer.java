// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.commands.arm.ArmLinear;
import frc.robot.commands.arm.ArmPosition;
import frc.robot.commands.arm.ArmStabilize;
import frc.robot.commands.camera.CameraChangePipeline;
import frc.robot.commands.drive.Linear;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.feed.FeedPercentage;
import frc.robot.commands.intake.IntakeBall;
import frc.robot.commands.intake.IntakePercentage;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private ShuffleboardTab turrettab = Shuffleboard.getTab("Turret");
    private NetworkTableEntry TurretClicks = turrettab.add("Turret Clicks", 0).getEntry();
      private NetworkTableEntry TurretDeg = turrettab.add("Turret Degrees", 0).getEntry();
  private ShuffleboardTab shootertab = Shuffleboard.getTab("Shoot");
    private NetworkTableEntry FrontRPM = shootertab.add("FShootInput", Vars.SHOOTER_FRONT_ESTIMATED_RPM).getEntry();
    private NetworkTableEntry PhysicalRPM = shootertab.add("FrontRPMOut",0).getEntry();
    private NetworkTableEntry BackSpinRPM = shootertab.add("BShootOutput",0).getEntry();
    private NetworkTableEntry BackSpinRPMINPUT = shootertab.add("BShootInput",Vars.SHOOTER_BACK_ESTIMATED_RPM).getEntry();
    private NetworkTableEntry FeedPercent = shootertab.add("Feed Percentage", 0).getEntry();
    private NetworkTableEntry IntakeBottomInput = shootertab.add("Intake Bottom Input", 0).getEntry();
    private NetworkTableEntry IntakeTopInput = shootertab.add("Intake Top Input", 0).getEntry();
    private NetworkTableEntry ShooterFrontPercentage = shootertab.add("SFPInput", Vars.SHOOTER_FRONT_ESTIMATED_PERCENTAGE).getEntry();
    private NetworkTableEntry ShooterBackPercentage = shootertab.add("SBPInput", Vars.SHOOTER_BACK_ESTIMATED_PERCENTAGE).getEntry();
    private ShuffleboardTab armtab = Shuffleboard.getTab("Arm");

  // The robot's subsystems and commands are defined here...
  protected static final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();
  protected static final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  protected static final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
  protected static final ArmSubsystem m_ArmPosition = new ArmSubsystem();
  protected static final FeedSubsystem m_FeedSubsystem = new FeedSubsystem();
  protected static final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  //Camera
  private final CameraChangePipeline m_TrackingCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem, CameraSubsystem.Tracking_Pipline);
  private final CameraChangePipeline m_DriverCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem, CameraSubsystem.Drive_Pipline);

  //Turret
  private final TurretRotate m_TurretRotate = new TurretRotate(m_TurretSubsystem, ()->IO.getXBoxRightX());
  private final TurretAim m_TurretAutoAim = new TurretAim(m_CameraSubsystem, m_TurretSubsystem);

  //Shooter TODO will need to be checked by Josh
  private final ShooterPrep m_ShootSequence = new ShooterPrep(m_IntakeSubsystem, m_FeedSubsystem);
  private final ShooterPercentage m_UnjamShooter = new ShooterPercentage(m_ShooterSubsystem, ()-> Vars.SHOOTER_FRONT_, ()-> Vars.SHOOTER_BACK_ESTIMATED_PERCENTAGE);

  //Intake
  private final IntakeBall m_IntakeSequence = new IntakeBall(m_IntakeSubsystem, m_FeedSubsystem);
  private final IntakePercentage m_UnjamIntake = new IntakePercentage(m_IntakeSubsystem, top, bottom);
  
  //Arm
  private final ArmLinear m_ArmLinear = new ArmLinear(m_ArmPosition, ()->IO.getXBoxLeftY());
  private final ArmStabilize m_ArmStabilize = new ArmStabilize(m_ArmPosition);
  private final TurretPreset m_TurretPreset90 = new TurretPreset(m_TurretSubsystem, 90);
  private final TurretPreset m_TurretPreset270 = new TurretPreset(m_TurretSubsystem, 270);
  private final TurretPreset m_TurretPreset0 = new TurretPreset(m_TurretSubsystem, 0);
  private final TurretPreset m_TurretPreset180 = new TurretPreset(m_TurretSubsystem, 180);
  private final ArmPosition m_Arm0 = new ArmPosition(m_ArmPosition, 0);
  private final ArmPosition m_ArmPickupBall = new ArmPosition(m_ArmPosition, 135);

  //Misc
  private final RunCommand m_DashWriter = new RunCommand(()-> WriteToDashboard()){public boolean runsWhenDisabled(){return true;}};
  //{public boolean isFinished(){return false;}};

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_tankDrive);
    
    CommandScheduler.getInstance().schedule(m_DashWriter);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    IO.rightJoystick_1.whenPressed(m_ShootSequence);
    IO.rightJoystick_2.whileHeld(m_TurretAutoAim);
    //IO.rightJoystick_3
    //IO.rightJoystick_4
    //IO.rightJoystick_5
    //IO.rightJoystick_6
    //IO.rightJoystick_7
    //IO.rightJoystick_8
    //IO.rightJoystick_9
    //IO.rightJoystick_10
    // Changes pipline to driver POV when pressed 
    IO.rightJoystick_11.whenPressed(m_DriverCameraChangePipeline);
    // Changes to Tracking POV when Pressed
    IO.rightJoystick_12.whenPressed(m_TrackingCameraChangePipeline);
    //IO.leftJoystick_1
    //IO.leftJoystick_2
    //IO.leftJoystick_3
    //IO.rightJoystick_4
    //IO.rightJoystick_5
    //IO.rightJoystick_6
    //IO.rightJoystick_7
    //IO.rightJoystick_8
    //IO.rightJoystick_9
    //IO.rightJoystick_10
    //IO.leftJoystick_11
    //IO.leftJoystick_12

    IO.xboxUp.whileHeld(m_Arm0);
    //IO.xboxRight
    IO.xboxDown.whenPressed(m_ArmPickupBall);
    //IO.xboxLeft
    IO.xbox_Y.whenPressed(m_TurretPreset0);
    IO.xbox_B.whenPressed(m_TurretPreset90);
    IO.xbox_A.whenPressed(m_TurretPreset180);
    IO.xbox_X.whenPressed(m_TurretPreset270);
    //IO.xbox_SELECT
    //IO.xbox_START
    IO.xbox_RB.whileHeld(m_IntakeSequence);
    IO.xbox_RT.whileHeld(m_UnjamShooter);
    //IO.xbox_LB
    IO.xbox_LT.whileHeld(m_UnjamIntake);
  }
  
  public void WriteToDashboard(){
    TurretClicks.setDouble(m_TurretSubsystem.getClicks());
    TurretDeg.setDouble(m_TurretSubsystem.getRotationDegrees());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new CommandBase(){
    };
  }
}
