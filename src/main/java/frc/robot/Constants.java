// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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

  //TODO fix Camera_ELEVATION
  public static final double Camera_ELEVATION = 31; //in
  public static final double Camera_TILT = 35.4; //deg

  //Field Constants (May be Incomplete) 
  public static final double BAR1 = 48.75; 
  public static final double BAR2 = 60.25; 
  public static final double BAR3 = 75.625; 
  public static final double BAR4 = 91; 
  public static final double VISION_TAPE = 101.625; //Height of vision tape off floor
  // Ball positions (origin at the hub, viewed from the angle of the scoring table https://www.desmos.com/calculator/han6ahdh5e)
  // Balls are counted going clockwise around the hub
  // Blue Balls
  public static final Translation2d BALL_BLUE_SIDE_BLUE_1 = new Translation2d(-25.11, -146.14);
  public static final Translation2d BALL_BLUE_SIDE_BLUE_2 = new Translation2d(-121.09, -85.58);
  public static final Translation2d BALL_BLUE_SIDE_BLUE_3 = new Translation2d(-125.41, 79.13);
  public static final Translation2d BALL_BLUE_STATION = new Translation2d(-277.73, -115.91);
  public static final Translation2d BALL_BLUE_SIDE_RED_1 = new Translation2d(-32.73, 144.63);
  public static final Translation2d BALL_BLUE_SIDE_RED_2 = new Translation2d(144.63, 32.73);
  public static final Translation2d BALL_BLUE_SIDE_RED_3 = new Translation2d(85.58, -121.09);
  // Red Balls
  public static final Translation2d BALL_RED_SIDE_BLUE_1 = new Translation2d(25.11, 146.14);
  public static final Translation2d BALL_RED_SIDE_BLUE_2 = new Translation2d(121.09, 85.58);
  public static final Translation2d BALL_RED_SIDE_BLUE_3 = new Translation2d(125.41, -79.13);
  public static final Translation2d BALL_RED_STATION = new Translation2d(32.73, -144.63);
  public static final Translation2d BALL_RED_SIDE_RED_1 = new Translation2d(277.73, 115.91);
  public static final Translation2d BALL_RED_SIDE_RED_2 = new Translation2d(-85.58, 121.09);
  public static final Translation2d BALL_RED_SIDE_RED_3 = new Translation2d(-144.63, -32.73);
}
