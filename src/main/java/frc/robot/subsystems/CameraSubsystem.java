// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  /** Limelight network table keyword */
	private static final String LIMELIGHT = "limelight";
	/** Whether the limelight has any valid targets (0 or 1) */
	private static final String TARGET_EXISTS = "tv";
	/** Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
	private static final String TARGET_X = "tx";
	/** Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
	private static final String TARGET_Y = "ty";
	/** Target Area (0% of image to 100% of image) */
	private static final String TARGET_AREA = "ta";
	/** Skew or rotation (-90 degrees to 0 degrees) */
	private static final String TARGET_SKEW = "ts";
	/** Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
	private static final String TARGET_WIDTH = "thor";
	/** Vertical sidelength of the rough bounding box (0 - 320 pixels) */
	private static final String TARGET_HEIGHT = "tvert";
// Initialize Network Table
  private NetworkTable CameraTable;
// Set Tracking Pipline to 1
  public static final int Tracking_Pipline = 1;
   // Driver POV
   public static final int Drive_Pipline = 0;

  public CameraSubsystem() {
    CameraTable =  NetworkTableInstance.getDefault().getTable(LIMELIGHT);
  }
  public boolean TargetExists(){
    return(CameraTable.getEntry(TARGET_EXISTS).getDouble(0) == 1 ? true: false);
  }
  public double GetTargetX(){
    return (CameraTable.getEntry(TARGET_X).getDouble(0));
    // LL2 Range: -29.8 to 29.8 degrees)
  }
  public double GetTargetY(){
    return(CameraTable.getEntry(TARGET_Y).getDouble(0));
    // LL2 Range: -24.85 to 24.85 degrees)
  }
  // retrieve Target Width
  public double getTargetWidth(){
    return(CameraTable.getEntry(TARGET_WIDTH).getDouble(0));
  }
  // Retrieves Target Height
  public double getTargetHeight(){
    return(CameraTable.getEntry(TARGET_HEIGHT).getDouble(0));
  }
  public void ChangePipeline(int pipe){
    CameraTable.getEntry("pipeline").setDouble(pipe);
  }

  public double getTargetDistance() {
    if(!TargetExists()) return -1;

    double numerator = Constants.Target_ELEVATION - Constants.Camera_ELEVATION;
    double radians = 
      Math.toRadians(
        Constants.Camera_TILT + CameraTable.getEntry(TARGET_Y).getDouble(0)
      );
    double denominator = Math.tan(radians);

    double distance = numerator / denominator;
    return distance;
    // See calculation on page 26 of programming notebook
    // Angle of the camera to the tower = Angle of the camera on the robot + the Angle the robot Sees
    // Distance= Height of the Tower - Height of the Robot / Tan(Angle of the Camera to the Tower)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
