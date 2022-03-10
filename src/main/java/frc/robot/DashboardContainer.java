/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A singleton tool to interface to the dashboard.
 */
public class DashboardContainer {

  private static DashboardContainer instance = null;

  private SendableChooser<Integer> verbosityChooser = new SendableChooser<Integer>();
  private NetworkTableEntry FrontRPMInput, BackRPMInput, FrontPercentInput, BackPercentInput, FrontBoostRPMInput, BackBoostRPMInput;
  private NetworkTableEntry FrontRPMOutput, BackRPMOutput, FeedContainsBall;

  // This constructor is private because it is a singleton
  private DashboardContainer() {}

  private void init() {
    getTab(TabsIndex.kDriver);
    getTab(TabsIndex.kAuto);
    getTab(TabsIndex.kConfig);
  }

  /**
   * Gets the DashboardContainer instance.
   */
  public static DashboardContainer getInstance() {
    if (instance==null) {
      instance = new DashboardContainer();
      instance.init();
    }
    return instance;
  }

  /**
   * An enum to reference a tab on the dashboard.
   */
  public enum TabsIndex {
    kSmartDashboard(0,"SmartDashboard"),
    kLiveWindow(1,"LiveWindow"),
    kDriver(2,"Driver"),
    kAuto(3,"Auto"),
    kConfig(4,"Config");
    public final int index;
    public final String name;
    TabsIndex(int index, String name) {
      this.index = index;
      this.name = name;
    }
    public int getIndex(){return index;}
    public String getName(){return name;}
  }

  /**
   * Set the shuffleboard to be looking at a specific tab.
   * @param index Desired tab.
   * @return The selected tab.
   * @see TabsIndex
   */
  public ShuffleboardTab setTab(TabsIndex tabindex) {
    Shuffleboard.selectTab(tabindex.getName());
    return Shuffleboard.getTab(tabindex.getName());
  }
  
  /**
   * Get a tab from the shuffleboard.
   * @param tabindex Desired tab.
   * @return The desired tab.
   * @see TabsIndex
   */
  public ShuffleboardTab getTab(TabsIndex tabindex) {
    return Shuffleboard.getTab(tabindex.getName());
  }

  /**
   * This should ONLY be called ONCE.
   * This should also be called AFTER the RobotContainer has created all of it's subsystems.
   * This is because this will make AutoContainer put items on the dashboard and make commands that involve
   * subsystems from the RobotContainer and if the robot container did not make the items then there will be
   * issues with null items.
   */
  public void boot() {
    setupDriver();
    setupConfig();
    AutoContainer.getInstance(); // this is to get the auto to do it's tab
    // config is handled in other code
  }

  private void setupDriver() {
    ShuffleboardTab driver = getTab(TabsIndex.kDriver);
    // this is a troubleshooting note: if the limelight stream does not appear right on the dashboard:
    // did you flash it? if so, did you remember to change the team number and static ip? if no, do that
    HttpCamera limelight = new HttpCamera("limelight", "http://10.24.78.11:5800");
    driver.add("Limelight", limelight).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0).withSize(5, 3);

    verbosityChooser.addOption("Silent", 0);
    verbosityChooser.setDefaultOption("Driver", 1);
    verbosityChooser.addOption("Driver Debug", 2);
    verbosityChooser.addOption("Middle", 3);
    verbosityChooser.addOption("Programmer", 4);
    verbosityChooser.addOption("Programmer Debug", 5);
    driver.add("Verbosity",verbosityChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(2, 3).withSize(2, 1);

    FrontRPMOutput = driver.add("Front RPM", 0).withPosition(7, 0).getEntry();
    BackRPMOutput = driver.add("Back RPM", 0).withPosition(7, 1).getEntry();
    FeedContainsBall = driver.add("Feed Contains Ball", false).withPosition(7, 2).getEntry();
    
    driver.addNumber("Xbox left Y", () -> IO.getXBoxLeftY()).withPosition(9, 0);
    driver.addNumber("Xbox right X", () -> IO.getXBoxRightX()).withPosition(9, 1);
    driver.addNumber("Joystick Right Y", () -> IO.getRightY()).withPosition(9, 2);
    FrontRPMInput = driver.add("Front Shooter RPM Input", Vars.SHOOTER_FRONT_DEFAULT_RPM).withPosition(0, 0).withSize(2, 1).getEntry();
    BackRPMInput = driver.add("Back Shooter RPM Input", Vars.SHOOTER_BACK_DEFAULT_RPM).withPosition(0, 1).withSize(2, 1).getEntry();
    FrontBoostRPMInput = driver.add("Front Boost RPM Input", Vars.SHOOTER_BOOST_FRONT_RPM).withPosition(0, 2).withSize(2, 1).getEntry();
    BackBoostRPMInput = driver.add("Back Boost RPM Input", Vars.SHOOTER_BOOST_BACK_RPM).withPosition(0, 3).withSize(2, 1).getEntry();
  }

  private void setupConfig() {
    ShuffleboardTab config = getTab(TabsIndex.kConfig);
    FrontPercentInput = config.add("Front Shooter Percent Input", Vars.SHOOTER_FRONT_ESTIMATED_PERCENTAGE).withPosition(4, 0).withSize(2, 1).getEntry();
    BackPercentInput = config.add("Back Shooter Percent Input", Vars.SHOOTER_BACK_ESTIMATED_PERCENTAGE).withPosition(4, 1).withSize(2, 1).getEntry();
  }

  public double FrontRPMInput()
  {
    return FrontRPMInput.getDouble(Vars.SHOOTER_FRONT_DEFAULT_RPM);
  }

  public double BackRPMInput()
  {
    return BackRPMInput.getDouble(Vars.SHOOTER_BACK_DEFAULT_RPM);
  }
  public double FrontBoostRPMInput()
  {
    return FrontBoostRPMInput.getDouble(Vars.SHOOTER_BOOST_FRONT_RPM);
  }

  public double BackBoostRPMInput()
  {
    return BackBoostRPMInput.getDouble(Vars.SHOOTER_BOOST_BACK_RPM);
  }

  public double FrontPercentInput()
  {
    return FrontPercentInput.getDouble(Vars.SHOOTER_FRONT_ESTIMATED_PERCENTAGE);
  }

  public double BackPercentInput()
  {
    return BackPercentInput.getDouble(Vars.SHOOTER_BACK_ESTIMATED_PERCENTAGE);
  }

  /**
   * Gets the verbosity level of the robot: <p>
   * 1 - Driver Info <p>
   * 2 - Driver Debug <p>
   * 3 - Middle (between 2 and 4) <p>
   * 4 - Programmer Info <p>
   * 5 - Programmer Debug <p>
   * 0 - Silent
   */
  public int getVerbosity() {
    // if there is a null value, then choose verbosity of 1 so something does not crash elsewhere
    return (verbosityChooser.getSelected() != null) ? verbosityChooser.getSelected() : 1;
  }
public void putDashboard(){
  switch (getVerbosity())
  {
    case 5:
      SmartDashboard.putNumber("Turret/Gain", RobotContainer.m_TurretSubsystem.getTurretGain());
      SmartDashboard.putNumber("Shooter/EncoderVelocity", RobotContainer.m_ShooterSubsystem.getEncVelocityFront());
      SmartDashboard.putNumber("Shooter/Encoder", RobotContainer.m_ShooterSubsystem.getEncFront());
      SmartDashboard.putNumber("Arm/Encoder", RobotContainer.m_ArmSubsytem.getEnc());
      SmartDashboard.putNumber("Turret/Clicks", RobotContainer.m_TurretSubsystem.getClicks());
      SmartDashboard.putNumber("Climb/EncoderClicks", RobotContainer.m_ClimbSubsystem.getEnc());
      SmartDashboard.putNumber("Climb/EncoderVelocity", RobotContainer.m_ClimbSubsystem.getEncVel());
    case 4:
      SmartDashboard.putNumber("Shooter/FrontGain", RobotContainer.m_ShooterSubsystem.getGainFront());
      SmartDashboard.putNumber("Shooter/BackGain", RobotContainer.m_ShooterSubsystem.getGainBack());
      SmartDashboard.putNumber("Arm/Gain", RobotContainer.m_ArmSubsytem.getGain());
      SmartDashboard.putNumber("Climb/Gain", RobotContainer.m_ClimbSubsystem.getGain());
    case 3:
      SmartDashboard.putNumber("Camera/TargetX", RobotContainer.m_CameraSubsystem.GetTargetX());
      SmartDashboard.putNumber("Camera/TargetY", RobotContainer.m_CameraSubsystem.GetTargetY());
      SmartDashboard.putNumber("Camera/TargetWidth", RobotContainer.m_CameraSubsystem.getTargetWidth());
      SmartDashboard.putNumber("Camera/TargetHeight", RobotContainer.m_CameraSubsystem.getTargetHeight());
    case 2:
      SmartDashboard.putNumber("Turret/Degrees", RobotContainer.m_TurretSubsystem.getRotationDegrees());
      SmartDashboard.putBoolean("Camera/TargetExists", RobotContainer.m_CameraSubsystem.TargetExists());
      SmartDashboard.putNumber("Camera/TargetDistance", RobotContainer.m_CameraSubsystem.getTargetDistance());
      SmartDashboard.putBoolean("Arm/HallEffect", RobotContainer.m_ArmSubsytem.getHallEffect());
      SmartDashboard.putNumber("Arm/Position", RobotContainer.m_ArmSubsytem.getPosition());
      // SmartDashboard.putNumber("Climb/Position", RobotContainer.m_ClimbSubsystem.getPosition());
    case 1:
      FrontRPMOutput.setDouble(RobotContainer.m_ShooterSubsystem.getRPMFront());
      BackRPMOutput.setDouble(RobotContainer.m_ShooterSubsystem.getRPMBack());
      FeedContainsBall.setBoolean(RobotContainer.m_FeedSubsystem.containsBall());
      // SmartDashboard.putNumber("Shooter/FrontRPM", RobotContainer.m_ShooterSubsystem.getRPMFront());
      // SmartDashboard.putNumber("Shooter/BackRPM", RobotContainer.m_ShooterSubsystem.getRPMBack());
      // SmartDashboard.putBoolean("Feed/ContainsBall", RobotContainer.m_FeedSubsystem.containsBall());
      break;
    case 0:
    default:
      break;
  }

  }

}
