package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ClimbSubsystem;

public class Vars {

  //TODO change values
  public static final double LINEAR_ANGLE_P = 0;

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
  public static final boolean CLIMB_MOTOR_REVERSED = false;
  
  
  // flipped concoders
  public static final boolean TURRET_ENCODER_REVERSED = false;
  public static final boolean ARM_ENCODER_REVERSED = false;
  public static final boolean CLIMB_ENCODER_REVERSED = false;

  // turret
  public static final double TURRET_MAX_ROTATION = 210 ; // degrees clockwise, clockwise bound // TODO ask josh about this value
  public static final double TURRET_MIN_ROTATION = -50; // degrees clockwise, counterclockwise bound
  public static final double TURRET_TOLERANCE = 2; // degrees
  public static final double TURRET_KP = 4;
  public static final double TURRET_180 = 180;
  public static final double TURRET_0 = 0;
  public static final double TURRET_RAMP_UP_TIME = 0.1; // Seconds 

  //TODO change values
  public static final double INTAKE_PERCENT = 1;
  public static final double FEED_REVERSED_PERCENT = -1;
  public static final double AUTO_FEED_REVERSED_PERCENT = -0.5; // FOR AUTO ONLY
  public static final double AUTO_INTAKE_TOP_REVERSED_PERCENT = -0.4; // FOR AUTO ONLY
  public static final double AUTO_INTAKE_BOTTOM_REVERSED_PERCENT = -0.4; // FOR AUTO ONLY
  public static final double INTAKE_TOP_REVERSED_PERCENT = -0.4;
  public static final double INTAKE_BOTTOM_REVERSED_PERCENT = -0.4;
  public static final double FEED_REVERSED_PERCENT_SLOW = -0.5;
  public static final double FEED_REVERSE_BUTTON = -0.4;
  /** output rotations per input rotations */
  public static final double SHOOTER_FRONT_GEARING = 40.0/54.0; // output rotations / input rotations
  public static final double SHOOTER_TOLERANCE = 200; // +rpm
  public static final double SHOOTER_PERCENT_TOLERANCE = 0.05; // rpm
  public static final double SHOOTER_PRE = 0.2; // percent (for hopper and feed to pulse back)
  public static final double SHOOTER_FEED = 1; // percent (for hopper and feed to feed)
  public static final double SHOOTER_SLOW_FEED = 0.2; // percent (for hopper and feed to feed when the shooter is not rev-ed)
  public static final double SHOOTER_SLOW_INTAKE = 0.2;
  public static final double SHOOTER_BACK_FEED_TIME = 0.2; // seconds
  public static final double AUTO_SHOOTER_BACK_FEED_TIME = .2; //seconds
  public static final double SHOOTER_FRONT_REVERSE = -0.3; // percent
  public static final double SHOOTER_BACK_REVERSE = -0.3; // percent
  public static final double SHOOTER_FRONT_DEFAULT_RPM = 2450; // rpm (on dashboard)
  public static final double SHOOTER_BACK_DEFAULT_RPM = 7500; // rpm (on dashboard)
  public static final double SHOOTER_BOOST_FRONT_RPM = 3500;
  public static final double SHOOTER_BOOST_BACK_RPM = 7500;
  public static final double SHOOTER_FRONT_KP = 0;
  public static final double SHOOTER_BACKSPIN_KP = 0.05;
  // below used to calculate feed forward of shooter wheels
  public static final double SHOOTER_FRONT_ESTIMATED_PERCENTAGE =  0.536; // using this percent... // 0.515
  public static final double SHOOTER_FRONT_ESTIMATED_RPM = 2413; // ...causes this rpm
  public static final double SHOOTER_BACK_ESTIMATED_PERCENTAGE = 0.825; // using this percent...
  public static final double SHOOTER_BACK_ESTIMATED_RPM = 7480; // ...causes this rpm
  // and therefore the estimated velocity is...
  public static final int SHOOTER_FRONT_NATIVE_ESTIMATED_VELOCITY = (int)(SHOOTER_FRONT_ESTIMATED_RPM / SHOOTER_FRONT_GEARING / 600 * Constants.CLICKS_PER_REV_INTEGRATED);
  public static final int SHOOTER_BACK_NATIVE_ESTIMATED_VELOCITY = (int)(SHOOTER_BACK_ESTIMATED_RPM / 600 * Constants.CLICKS_PER_REV_QUADRATURE);
  // end of ff calculation

  //Arms
  public static final double ARM_GEARING = 36.0/54.0; // output rotations / input rotations // XXX updated Arm
  public static final double ARM_MINIMUM_ANGLE = -1; // degrees
  public static final double ARM_MAXIMUM_ANGLE = 123; // degrees
  public static final double ARM_TOLERANCE = 3; // degrees
  public static final double ARM_P = 0.75;
  public static final double ARM_ANGLE_PICKUP = 120;
  public static final double ARM_ZERO_VOLTAGE = -0.2;
  public static final double ARM_IN = 5;
// Auto
public static final double WHEEL_DIAMETER = 6; // inches
public static final double MAX_VELOCITY = 75; // inches/sec
public static final double MAX_ACCELERATION = 150; // inches/sec^2
/**
 * Equivilant to 1/Gear Ratio.
 * Use this to convert from 1 rotation of the motor to 1 rotation of the output shaft: input * GEARING = output.
 */
public static final double DRIVETRAIN_GEARING = 1/10.91;
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
 public static final double AUTO_WAIT_TO_SHOOT_TIME_LONG = 4;
 public static final double AUTO_RUNNING_INTAKE = 10;
 public static final double AUTO_FRONT_SHOOTER_RPM = 1650; //rpm
 public static final double AUTO_BACK_SHOOTER_RPM= 7500; //rpm
 // public static final double DRIVE_MAX_M_PER_S =189.72441; // in/s //XXX find this
 // public static final double DRIVE_MAX_M_PER_S_SQUARED = 1743; // in/s^2 //XXX find this value
 public static final double AUTO_MAX_M_PER_S = 25; // in/s XXX ???
 public static final double AUTO_MAX_M_PER_S_SQUARED = 10; // in/s^2 XXX ???
// Distances
 public static final double AUTO_SHOOT_MOVE_DISTANCE = 60; //in
 public static final double AUTO_FORWARD_DISTANCE = 30; // in
 public static final double AUTO_BACK_FORWARD = 60; // in
 public static final double AUTO_INTAKE_BALL_BACKWARD_DISTANCE = -48; // in
 public static final double AUTO_INTAKE_BALL_FORWARD_DISTANCE= 51; // in
 public static final double AUTO_SHOOT_FRONT_SPEED_FOR_AUTOINTAKESHOOT = 2575; // rpm
 public static final double AUTO_SHOOT_RAMP_TIME = 2; // second
 
  // Pathing 
public static final double DRIVE_KS = 0.62263; // Volts
public static final double DRIVE_KV = 2.4428; // Volts * s/m
public static final double DRIVE_KA = 0.29807; // Volts * s^2/m
public static final double TRACK_WIDTH = Units.inchesToMeters(26.255); // meters
public static final double AUTO_PATH_KP = 3.2308;
public static final DifferentialDriveKinematics KINEMATICS =
  new DifferentialDriveKinematics(TRACK_WIDTH);

  // public static final int CLIMB_TRACK_DIAMETER = 55; // XXX ?
  // public static final int CLIMB_GEARING = 26; // XXX ?

  public static final double CLIMB_KP = 0.16; // XXX ?
  public static final double CLIMB_KI = 0; // XXX ?
  public static final double CLIMB_KD = 0; // XXX ?
  public static final double CLIMB_KF = 0.15; // XXX ?
  public static final double CLIMB_TOLERANCE = 2000; //+- native
  // XXX below for magic
  // public static final double CLIMB_SPEED = 12; // inches / sec
  // public static final double CLIMB_ACCELERATION = 6; // inches / sec^2
  // public static final double CLIMB_NATIVE_PER100MS = ClimbSubsystem.toClicks(CLIMB_SPEED*0.1); //Native units/100 ms
  // public static final double CLIMB_NATIVE_PER_100MS_PER_SEC = ClimbSubsystem.toClicks(CLIMB_ACCELERATION*0.1); //(Native units/100 ms)/sec
  public static final double CLIMB_NATIVE_PER100MS = 94149; //Native units/100 ms
  public static final double CLIMB_NATIVE_PER_100MS_PER_SEC = 150000; //(Native units/100 ms)/sec

  public static final int SMOOTHING = 0; // XXX for magic

  public static final double CLIMB_FINISH_PERCENT = -0.3;
  public static final double CLIMB_DOWN = 2000;
  public static final double CLIMB_UP = 204000; //native TODO change later
  public static final double CLIMB_OUT_AND_UP = 40000; // native (used when the pistons are put out so the climb does not get stuck) TODO change later

  public static final double PNEUMATIC_LOOP_COUNT = 5;

  // TODO make this not follow after the competition
  public static final double CLIMB_MINIMUM = CLIMB_DOWN;
  public static final double CLIMB_MAXIMUM = CLIMB_UP; // native
}
