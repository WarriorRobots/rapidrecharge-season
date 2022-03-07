package frc.robot;

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
  public static final boolean CLIMB_MOTOR_REVERSED = false;

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
  public static final double SHOOTER_BACK_CLICKS_PER_REV = 4096.0;
  public static final double SHOOTER_FRONT_CLICKS_PER_REV = 4096.0; //TODO check
  /** output rotations per input rotations */
  public static final double SHOOTER_FRONT_GEARING = 4.0/5.0; 
  public static final double SHOOTER_DEFAULT = 0; //rpm
  public static final double SHOOTER_TOLERANCE = 200; // +rpm
  public static final double SHOOTER_KP = 0.15;
  public static final double SHOOTER_PRE = -0.2; // percent (for hopper and feed to pulse back)
  public static final double SHOOTER_FEED = 1; // percent (for hopper and feed to feed)
  public static final double SHOOTER_SLOW_FEED = 0.2; // percent (for hopper and feed to feed when the shooter is not rev-ed)
  public static final double SHOOTER_INTAKE_AGITATE = 0.25; // percent (to agitate balls with the intake)

  //Arms
  public static final double ARM_MINIMUM_ANGLE = -5; // degrees
  public static final double ARM_MAXIMUM_ANGLE = 265; // degrees
  public static final double ARM_TOLERANCE = 3; // degrees  

  /** Typical motor output as percent */
  public static final double SHOOTER_FRONT_ESTIMATED_VOLTAGE = 0; // TODO move this to Vars
  /** Velocity of shooter in native units per 100ms at typical motor output (at the encoder) */
  public static final int SHOOTER_FRONT_NATIVE_ESTIMATED_VELOCITY = 0; // TODO move this to Vars
  
  /** Typical motor output as percent */
  public static final double SHOOTER_BACK_ESTIMATED_VOLTAGE = 0; // TODO move this to Vars
  /** Velocity of shooter in native units per 100ms at typical motor output (at the encoder) */
  public static final int SHOOTER_BACK_NATIVE_ESTIMATED_VELOCITY = 0; // TODO move this to Vars
  
  public static final int CLIMB_MINIMUM = 30; //TODO Adjust as some point
  public static final int CLIMB_MAXIMUM = 60; //TODO Adjust as some point

  public static final int CLIMB_TRACK_DIAMETER = 55; //TODO adjust at some point
  public static final int CLIMB_GEARING = 26; //TODO adjust at some point
  public static final int CLIMB_CLICKS_PER_REV = 300; //TODO adjust at some point

  public static final double CLIMB_KP = 0.01; //TODO adjust at somepoint
  public static final double CLIMB_KI = 0; //TODO adjust at somepoint
  public static final double CLIMB_KD = 0; //TODO adjust at somepoint
  public static final double CLIMB_TOLERANCE = 1; //+- inches
}
