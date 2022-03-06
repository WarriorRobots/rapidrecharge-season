// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int PRIMARY_PID = 0; // primary pid ids
  public static final int AUXILARY_PID = 1; // auxilary pid ids
  public static final int MS_TIMEOUT = 10; // 10 ms before a talon config fails

  //Sensor resolution see: https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
  public static final double CLICKS_PER_REV_QUADRATURE = 4096.0;
  public static final double CLICKS_PER_REV_INTEGRATED = 2048.0; 

  //Limelight2
  public static final double PIXELS_H = 320;
  public static final double PIXELS_V = 240;
	public static final double RAD_H = 1.04;
	public static final double RAD_V = 0.867;
	public static final double PPR_H = PIXELS_H / RAD_H; // Pixels per Radian
	public static final double PPR_V = PIXELS_V / RAD_V; // Pixels per Radian
  public static final double Target_ELEVATION = 104; //in

  //TODO change Camera_ELEVATION
  public static final double Camera_ELEVATION = 0; //in
  public static final double Camera_TILT = 35.4; //deg

  //PixyCam
  public static final double PIXY_FOV_HORIZ = 70; // horizontal FOV in degrees
  public static final double PIXY_FOV_VERT = 47; // vertical FOV in degrees

  //Field Constants (May be Incomplete) 
  public static final double BAR1 = 48.75; 
  public static final double BAR2 = 60.25; 
  public static final double BAR3 = 75.625; 
  public static final double BAR4 = 91; 
  public static final double VISION_TAPE = 101.625; //Height of vision tape off floor
}
