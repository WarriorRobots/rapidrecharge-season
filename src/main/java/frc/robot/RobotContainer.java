// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.arm.ArmLinear;
import frc.robot.commands.arm.ArmPosition;
import frc.robot.commands.camera.CameraChangePipeline;
//import frc.robot.commands.drive.Linear;
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
import frc.robot.commands.turret.TurretUnsafeRotate;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private ShuffleboardTab tab = Shuffleboard.getTab("Turret");
  private NetworkTableEntry TurretClicks=
     tab.add("Turret Clicks", 0)
        .getEntry();
        private NetworkTableEntry TurretDeg=
     tab.add("Turret Degrees", 0)
        .getEntry();
  private ShuffleboardTab tab1 = Shuffleboard.getTab("Shoota");
    private NetworkTableEntry FrontRPM=
        tab1.add("Front RPM Input", 0)
          .getEntry();
    private NetworkTableEntry PhysicalRPM=
        tab1.add("Front Physical RPM",0)
          .getEntry();
    private NetworkTableEntry BackSpinRPM =
    tab1.add("Back Spin Output",0)
        .getEntry();
    private NetworkTableEntry BackSpinRPMINPUT =
      tab1.add("Back Spin Input",0)
            .getEntry();
   private NetworkTableEntry FeedPercent =
      tab1.add("Feed Percentage", 0)
          .getEntry();
          private NetworkTableEntry IntakeBottomInput =
      tab1.add("Intake Bottom Input", 0)
          .getEntry();
          private NetworkTableEntry IntakeTopInput =
      tab1.add("Intake Top Input", 0)
          .getEntry();
    private NetworkTableEntry ShooterFrontPercentage =
      tab1.add("Shooter Front Percent Input", 0)
          .getEntry();
          private NetworkTableEntry ShooterBackPercentage =
      tab1.add("Shooter Back Percent Input", 0)
          .getEntry();
          private NetworkTableEntry InfraredSensor =
          tab1.add("Ball Detection",0)
          .getEntry();
          private NetworkTableEntry HallEffect =
          tab1.add("HallEffect Detection",0)
          .getEntry();
   

  
    
  // The robot's subsystems and commands are defined here...
  protected static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  protected static final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();
  protected static final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
  protected static final ArmSubsystem m_ArmSubsytem = new ArmSubsystem();
  protected static final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  protected static final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  protected static final FeedSubsystem m_FeedSubsystem = new FeedSubsystem();

// Drivetrain
  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, ()->IO.getLeftY(), ()->IO.getRightY());
  //private final Linear m_linear = new Linear(m_drivetrain);
  private final SequentialCommandGroup m_shooterSequence = new SequentialCommandGroup(
    new ShooterPrep(m_IntakeSubsystem, m_FeedSubsystem),
    new ShooterFeed(m_ShooterSubsystem, m_IntakeSubsystem, m_FeedSubsystem){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}}
  ){public void end(boolean interrupted){m_IntakeSubsystem.stop();;m_FeedSubsystem.stop();}}; // This is to stop the hopper and feed if the command is stopped however not stop the shooter, that is handled by UnRev
  /** Clears the shooter and runs the shooter at an rpm for the shooter to then be fed */
  private final SequentialCommandGroup m_revTrigger = new SequentialCommandGroup(
    new ShooterPrep(m_IntakeSubsystem, m_FeedSubsystem),
    new ShooterRPM(m_ShooterSubsystem,()->FrontRPM.getDouble(0),()->BackSpinRPMINPUT.getDouble(0) ){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}}
  );
// Camera
  private final CameraChangePipeline m_TrackingCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem, CameraSubsystem.Tracking_Pipline);
  private final CameraChangePipeline m_DriverCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem, CameraSubsystem.Drive_Pipline);

  // TODO robot should be able to shoot balls high using values from the dashboard; see: https://github.com/WarriorRobots/dummyRobot2022/blob/aeeb984605764853f1215d135f280ca5627459ee/src/main/java/frc/robot/RobotContainer.java#L36

  // Turret
 // private final TurretRotate m_TurretRotate = new TurretRotate(m_TurretSubsystem, ()->IO.getXBoxLeftX());
  private final TurretUnsafeRotate m_TurretUnsafeRotate = new TurretUnsafeRotate(m_TurretSubsystem, ()->IO.getXBoxLeftY());
  private final TurretPreset m_TurretPreset90 = new TurretPreset(m_TurretSubsystem, 90);
  private final TurretPreset m_TurretPresetMinus90 = new TurretPreset(m_TurretSubsystem, -90);
  private final TurretPreset m_TurretPreset0 = new TurretPreset(m_TurretSubsystem, 0);
  private final TurretAim m_TurretAim = new TurretAim(m_CameraSubsystem, m_TurretSubsystem);//{public boolean isFinished(){return false;}};
  //Arm
  private final ArmLinear m_ArmLinear = new ArmLinear(m_ArmSubsytem, ()->IO.getXBoxLeftY());
  private final ArmPosition m_ArmPosition0= new ArmPosition(m_ArmSubsytem, 0);
   private final ArmPosition m_ArmPosition45= new ArmPosition(m_ArmSubsytem, 45);
    private final ArmPosition m_ArmPosition90 = new ArmPosition(m_ArmSubsytem, 90);
  
  // Write to DashBoard
  private final RunCommand m_DashWriter = new RunCommand(()-> WriteToDashboard()){public boolean runsWhenDisabled(){return true;}};
  // Feed
  private final FeedPercentage m_FeedPercentage = new FeedPercentage(m_FeedSubsystem, 1.0);
  private final FeedPercentage m_FeedPercentageBack = new FeedPercentage(m_FeedSubsystem, -1.0);
  // Intake
  private final IntakeBall m_IntakeBallForward = new IntakeBall(m_IntakeSubsystem, m_FeedSubsystem,1.0);
  private final IntakeBall m_IntakeBallBack = new IntakeBall(m_IntakeSubsystem, m_FeedSubsystem, -1.0);
  private final IntakePercentage m_IntakePercentage = new IntakePercentage(m_IntakeSubsystem,-1.0, 1.0);
  // Shooter
   private final ShooterRPM m_ShooterRPM = new ShooterRPM(m_ShooterSubsystem,()-> FrontRPM.getDouble(0),()->BackSpinRPMINPUT.getDouble(0));
   private final ShooterPercentage m_ShooterPercent = new ShooterPercentage(m_ShooterSubsystem, -0.50, 0.50);


  



  /** The container for the robot. Conj
   * tains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_tankDrive);
    CommandScheduler.getInstance().schedule(m_DashWriter);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Changes pipline to driver POV when pressed 
    // IO.leftJoystick_10.whenPressed(m_DriverCameraChangePipeline);
    // Changes to Tracking Josh POV when Pressed
    // IO.leftJoystick_11.whenPressed(m_TrackingCameraChangePipeline);
    // IO.leftJoystick_12.whileHeld(m_linear);
    IO.xbox_Y.whileHeld(m_ArmLinear);
    IO.xbox_RT.whileHeld(m_ShooterPercent);
    IO.xbox_LT.whileHeld(m_IntakePercentage);
    IO.xbox_LB.whileHeld(m_IntakeBallBack);
    IO.xbox_RB.whileHeld(m_FeedPercentage);
    IO.xbox_A.whileHeld(m_ArmPosition0);
    IO.xbox_B.whileHeld(m_ArmPosition45);
    IO.xbox_X.whileHeld(m_ArmPosition90);
    IO.xbox_SELECT.whileHeld(m_FeedPercentageBack);
    IO.rightJoystick_1.whileHeld(m_shooterSequence,false);

    
    


    // TODO a button should turn on the shooter

    // IO.xbox_RB.whileHeld(m_TurretUnsafeRotate);
    // IO.xbox_Y.whenPressed(m_TurretPreset0);
     //IO.xbox_START.whenPressed(m_TurretRotate);
    // IO.xbox_X.whenPressed(m_TurretPresetMinus90);
    // IO.xbox_A.whileHeld(m_TurretAim);
  }

  public void WriteToDashboard(){
    TurretClicks.setDouble(m_TurretSubsystem.getClicks());
    TurretDeg.setDouble(m_TurretSubsystem.getRotationDegrees());
    PhysicalRPM.setDouble(m_ShooterSubsystem.getRPMFront());
    BackSpinRPM.setDouble(m_ShooterSubsystem.getRPMBack());
    InfraredSensor.setBoolean(m_FeedSubsystem.containsBall());
    HallEffect.setBoolean(m_ArmSubsytem.getHallEffect());
        
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
