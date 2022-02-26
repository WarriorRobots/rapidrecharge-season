// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.camera.CameraChangePipeline;
import frc.robot.commands.drive.Linear;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretPreset;
import frc.robot.commands.turret.TurretRotate;
import frc.robot.commands.turret.TurretUnsafeRotate;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
  // The robot's subsystems and commands are defined here...
  protected static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  protected static final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();
  protected static final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();


  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, ()->IO.getLeftY(), ()->IO.getRightY());
  private final Linear m_linear = new Linear(m_drivetrain);

  private final CameraChangePipeline m_TrackingCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem, CameraSubsystem.Tracking_Pipline);
  private final CameraChangePipeline m_DriverCameraChangePipeline = new CameraChangePipeline(m_CameraSubsystem, CameraSubsystem.Drive_Pipline);

  // TODO robot should be able to shoot balls high using values from the dashboard; see: https://github.com/WarriorRobots/dummyRobot2022/blob/aeeb984605764853f1215d135f280ca5627459ee/src/main/java/frc/robot/RobotContainer.java#L36

  private final TurretRotate m_TurretRotate = new TurretRotate(m_TurretSubsystem, ()->IO.getXBoxLeftX());
  private final TurretUnsafeRotate m_TurretUnsafeRotate = new TurretUnsafeRotate(m_TurretSubsystem, ()->IO.getXBoxLeftY());
  private final TurretPreset m_TurretPreset90 = new TurretPreset(m_TurretSubsystem, 90);
  private final TurretPreset m_TurretPresetMinus90 = new TurretPreset(m_TurretSubsystem, -90);
  private final TurretPreset m_TurretPreset0 = new TurretPreset(m_TurretSubsystem, 0);
  private final TurretAim m_TurretAim = new TurretAim(m_CameraSubsystem, m_TurretSubsystem);//{public boolean isFinished(){return false;}};

  private final RunCommand m_DashWriter = new RunCommand(()-> WriteToDashboard()){public boolean runsWhenDisabled(){return true;}};

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
    IO.leftJoystick_10.whenPressed(m_DriverCameraChangePipeline);
    // Changes to Tracking Josh POV when Pressed
    IO.leftJoystick_11.whenPressed(m_TrackingCameraChangePipeline);
    IO.leftJoystick_12.whileHeld(m_linear);

    // TODO a button should turn on the shooter

    IO.xbox_RB.whileHeld(m_TurretUnsafeRotate);
    IO.xbox_Y.whenPressed(m_TurretPreset0);
    IO.xbox_B.whenPressed(m_TurretPreset90);
    IO.xbox_X.whenPressed(m_TurretPresetMinus90);
    IO.xbox_A.whileHeld(m_TurretAim);
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
