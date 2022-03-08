package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
/**
 * All variables tied to the physical behavior of the robot. Tuning, reverse booleans, etc.
 */
public class Vars {
  // robot characteristics
  public static final double WHEEL_DIAMETER = 6; // inches
  public static final double MAX_VELOCITY = 75; // inches/sec
  public static final double MAX_ACCELERATION = 150; // inches/sec^2
  public static final StatorCurrentLimitConfiguration DRIVETRAIN_CURRENTLIMIT = // Current limiting applied to the drivetrain
    new StatorCurrentLimitConfiguration(
      // limiting?, limit (A), threshold (A), threshold time (s)
      true, 60, 70, 0.5
    );
  
  // flipped motors
  public static final boolean LEFT_DRIVE_INVERTED = true;
  public static final boolean RIGHT_DRIVE_INVERTED = false;
  public static final boolean TURRET_REVERSED = false;
  public static final boolean SHOOTER_LEFT_REVERSED = false;
  public static final boolean SHOOTER_RIGHT_REVERSED = true;
  public static final boolean SHOOTER_BACK_INVERTED = true;
  public static final boolean ARM_REVERSED = false;
  public static final boolean FEED_REVERSED = false;
  public static final boolean INTAKE_TOP_REVERSED = false;
  public static final boolean INTAKE_BOTTOM_REVERSED = true;
  
  // flipped concoders
  public static final boolean TURRET_ENCODER_REVERSED = false;
  public static final boolean ARM_ENCODER_REVERSED = false;
  // Drivetrain
  /**
   * Equivilant to 1/Gear Ratio.
   * Use this to convert from 1 rotation of the motor to 1 rotation of the output shaft: input * GEARING = output.
   */
  public static final double DRIVETRAIN_GEARING = 1/10.91;
  // turret
  public static final double TURRET_MAX_ROTATION = 210 ; // degrees clockwise, clockwise bound // TODO ask josh about this value
  public static final double TURRET_MIN_ROTATION = -50; // degrees clockwise, counterclockwise bound
  public static final double TURRET_TOLERANCE = 2; // degrees
  public static final double TURRET_KP = 10;

  //Intake Values
  public static final double INTAKE_PERCENT = 1;
  public static final double FEED_REVERSED_PERCENT = -1;
  public static final double FEED_REVERSED_PERCENT_SLOW = -0.2;
  // Shooter
  /** output rotations per input rotations */
  public static final double SHOOTER_FRONT_GEARING = 40.0/54.0; // output rotations / input rotations
  /**Tolerance of Shooter */
  public static final double SHOOTER_TOLERANCE = 200; // +rpm
  /** KP of shooter, set to 0 now */
  public static final double SHOOTER_FRONT_KP = 0;//0.003;
  /** Back Shooter KP currently set to 0 */
  public static final double SHOOTER_BACKSPIN_KP = 0;// 0.0015;
  /**The Percent at which the feed runs */
  public static final double SHOOTER_PRE = 0.2; // percent (for hopper and feed to pulse back)
  /**Percent of Feed when shooting */
  public static final double SHOOTER_FEED = 1; // percent (for hopper and feed to feed)
  /** SLow feed to prepare shooter */
  public static final double SHOOTER_SLOW_FEED = 0.2; // percent (for hopper and feed to feed when the shooter is not rev-ed)
  /**Slow intake to prepare shooter */
  public static final double SHOOTER_SLOW_INTAKE = 0.2;
  public static final double SHOOTER_BACK_FEED_TIME = 0.25; // seconds
  // Estimated Shooter Output
  /**Front shoote estimated RPM */
  public static final double SHOOTER_FRONT_DEFAULT_RPM = 2300;
  /**Back SHooter  Estimated RPM */
    public static final double SHOOTER_BACK_DEFAULT_RPM = 6000;

  /** Typical motor output as percent */
  public static final double SHOOTER_FRONT_ESTIMATED_PERCENTAGE =  0.50;
  public static final double SHOOTER_FRONT_REVERSE = -0.3;
  public static final double SHOOTER_BACK_REVERSE = -0.3;
  public static final double SHOOTER_FRONT_ESTIMATED_RPM = 2333;
  public static final double SHOOTER_BACK_ESTIMATED_RPM = 5980;
  /** Velocity of shooter in native units per 100ms at typical motor output (at the encoder) */
  public static final int SHOOTER_FRONT_NATIVE_ESTIMATED_VELOCITY = (int)(SHOOTER_FRONT_ESTIMATED_RPM / SHOOTER_FRONT_GEARING / 600 * Constants.CLICKS_PER_REV_INTEGRATED);
  // the above uses the gearing because the RPM is of the flywheel where as the calculated native velocity is of the motor 
  
  /** Typical motor output as percent */
  public static final double SHOOTER_BACK_ESTIMATED_PERCENTAGE = 0.67;
  /** Velocity of shooter in native units per 100ms at typical motor output (at the encoder) */
  public static final int SHOOTER_BACK_NATIVE_ESTIMATED_VELOCITY = (int)(SHOOTER_BACK_ESTIMATED_RPM / 600 * Constants.CLICKS_PER_REV_QUADRATURE);
  //Arm
  public static final double ARM_GEARING = 36.0/54.0; // output rotations / input rotations // XXX updated Arm
  public static final double ARM_MINIMUM_ANGLE = -1; // degrees
  public static final double ARM_MAXIMUM_ANGLE = 123; // degrees
  public static final double ARM_TOLERANCE = 3.5; // degrees
  public static final double ARM_P = 0.75;
  /** Angle arm goes to pickup ball */
  public static final double ARM_ANGLE_PICKUP = 120; // Degrees
  public static final double ARM_ZERO_VOLTAGE = -0.2;
  public static final double ARM_IN = 5;
  // Pathing 
  public static final double DRIVE_KS = 0.62263; // Volts
  public static final double DRIVE_KV = 2.4428; // Volts * s/m
  public static final double DRIVE_KA = 0.29807; // Volts * s^2/m
  public static final double TRACK_WIDTH = Units.inchesToMeters(26.255); // meters
  public static final double AUTO_PATH_KP = 3.2308;
  public static final DifferentialDriveKinematics KINEMATICS =
    new DifferentialDriveKinematics(TRACK_WIDTH);
  // Reasonable baseline values for a RAMSETE follower in units of meters and 
  //Ramsete
  public static final double RAMSETE_B = 2;
  public static final double RAMSETE_ZETA = 0.7;
  
  // auto
  public static final double AUTO_SPEED_TOLERANCE = 0.1; // inches per sec
  public static final double AUTO_LINEAR_TOLERANCE = 2; // inches
  public static final double AUTO_LINEAR_P = 0.1;
  public static final double AUTO_LINEAR_I = 0;
  public static final double AUTO_LINEAR_D = 0;
  public static final double AUTO_LINEAR_ANGLE_P = 0.02;
  public static final double AUTO_ANGULAR_TOLERANCE = 2; // degrees
  public static final double AUTO_ANGULAR_P = 0.01;
  public static final double AUTO_ANGULAR_I = 0;
  public static final double AUTO_ANGULAR_D = 0;
  public static final double AUTO_WAIT_TO_SHOOT_TIME = 2;
  // public static final double DRIVE_MAX_M_PER_S =189.72441; // in/s //XXX find this
  // public static final double DRIVE_MAX_M_PER_S_SQUARED = 1743; // in/s^2 //XXX find this value
  public static final double AUTO_MAX_M_PER_S = 25; // in/s XXX ???
  public static final double AUTO_MAX_M_PER_S_SQUARED = 10; // in/s^2 XXX ???

  public static final double AUTO_BACKUP_DISTANCE = -60; //in
  public static final double AUTO_FORWARD_DISTANCE = 30;

}
