package frc.robot;

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
  
  // flipped concoders
  public static final boolean TURRET_ENCODER_REVERSED = false;
  public static final boolean ARM_ENCODER_REVERSED = false;

  // turret
  public static final double TURRET_MAX_ROTATION = 210 ; // degrees clockwise, clockwise bound // TODO ask josh about this value
  public static final double TURRET_MIN_ROTATION = -50; // degrees clockwise, counterclockwise bound
  public static final double TURRET_TOLERANCE = 2; // degrees
  public static final double TURRET_KP = 10;

  //TODO change values
  public static final double INTAKE_PERCENT = 1;
  public static final double FEED_REVERSED_PERCENT = -1;
  public static final double FEED_REVERSED_PERCENT_SLOW = -0.2;
  /** output rotations per input rotations */
  public static final double SHOOTER_FRONT_GEARING = 40.0/54.0; // output rotations / input rotations
  public static final double SHOOTER_TOLERANCE = 200; // +rpm
  public static final double SHOOTER_FRONT_KP = 0;//0.003;
  public static final double SHOOTER_BACKSPIN_KP = 0;// 0.0015;
  public static final double SHOOTER_PRE = 0.2; // percent (for hopper and feed to pulse back)
  public static final double SHOOTER_FEED = 1; // percent (for hopper and feed to feed)
  public static final double SHOOTER_SLOW_FEED = 0.2; // percent (for hopper and feed to feed when the shooter is not rev-ed)
   public static final double SHOOTER_SLOW_INTAKE = 0.2;
   public static final double SHOOTER_BACK_FEED_TIME = 0.25; // seconds
  //Arms
  public static final double ARM_GEARING = 36.0/54.0; // output rotations / input rotations // XXX updated Arm

  public static final double ARM_MINIMUM_ANGLE = -1; // degrees
  public static final double ARM_MAXIMUM_ANGLE = 123; // degrees
  public static final double ARM_TOLERANCE = 3.5; // degrees
  public static final double ARM_P = 0.75;
  public static final double ARM_ANGLE_PICKUP = 120;
  public static final double ARM_ZERO_VOLTAGE = -0.2;
  public static final double ARM_IN = 5;
  public static final double ARM_AWAY = 10;




  // Estimated Shooter Output
  /**Front shoote estimated RPM */
    public static final double SHOOTER_FRONT_DEFAULT_RPM = 2400;
  /**Back SHooter  Estimated RPM */
    public static final double SHOOTER_BACK_DEFAULT_RPM = 7500;

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


}
