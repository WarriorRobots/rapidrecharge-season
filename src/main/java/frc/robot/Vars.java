package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Vars {

  //TODO change values
    public static final double LINEAR_ANGLE_P = 0;

  // flipped motors
  public static final boolean LEFT_DRIVE_INVERTED = false;
  public static final boolean RIGHT_DRIVE_INVERTED = true;
  public static final boolean TURRET_REVERSED = false;
  public static final boolean SHOOTER_LEFT_REVERSED = true;
  public static final boolean SHOOTER_RIGHT_REVERSED = false;
  public static final boolean SHOOTER_BACK_INVERTED = false;

  // flipped concoders
  public static final boolean TURRET_ENCODER_REVERSED = false;

  // turret
  public static final double MAX_ROTATION = 175; // degrees clockwise
  public static final double MIN_ROTATION = -175; // degrees clockwise
  public static final double TURRET_TOLERANCE = 2; // degrees
  public static final double TURRET_KP = 10;

  //TODO change values
  public static final double INTAKE_PERCENT = 1;

  //shooter
  public static final double SHOOTER_DEFAULT = 0; //rpm
  // XXX find good +- value for tolerance
  public static final double SHOOTER_TOLERANCE = 200; // +rpm
  public static final double SHOOTER_KP = 0.15;
  public static final double SHOOTER_PRE = -0.2; // percent (for hopper and feed to pulse back)
  public static final double SHOOTER_FEED = 1; // percent (for hopper and feed to feed)
  public static final double SHOOTER_SLOW_FEED = 0.2; // percent (for hopper and feed to feed when the shooter is not rev-ed)
  public static final double SHOOTER_INTAKE_AGITATE = 0.25; // percent (to agitate balls with the intake)
  public static final double SHOOTER_BACK_ESTIMATED_VOLTAGE = 0.3;

  //Arms
  public static final double ARM_MINIMUM_ANGLE = -5; // degrees
  public static final double ARM_MAXIMUM_ANGLE = 265; // degrees
  public static final double ARM_TOLERANCE = 3; // degrees  

  //Units
  public static final double CLICKS_PER_REV = 4096.0;
  // Pathing 
  public static final double DRIVE_KS = 0; // Volts
  public static final double DRIVE_KV = 0; // Volts * s/m
  public static final double DRIVE_KA = 0; // Volts * s^2/m
  public static final double TRACK_WIDTH = 0; // meters
  public static final double AUTO_PATH_KP = 0;
  public static final DifferentialDriveKinematics KINEMATICS =
    new DifferentialDriveKinematics(TRACK_WIDTH);
  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double RAMSETE_B = 2;
  public static final double RAMSETE_ZETA = 0.7;
  //Ramsete

  // auto
  public static final double AUTO_LINEAR_TOLERANCE = 2; // inches
  public static final double AUTO_LINEAR_P = 0.03;
  public static final double AUTO_LINEAR_I = 0;
  public static final double AUTO_LINEAR_D = 0;
  public static final double AUTO_LINEAR_ANGLE_P = 0.02;

  public static final double AUTO_ANGULAR_TOLERANCE = 2; // degrees
  public static final double AUTO_ANGULAR_P = 0.01;
  public static final double AUTO_ANGULAR_I = 0;
  public static final double AUTO_ANGULAR_D = 0;

  public static final double AUTO_MAX_M_PER_S_SQUARED = 0;

  public static final double AUTO_MAX_M_PER_S = -0;

}
