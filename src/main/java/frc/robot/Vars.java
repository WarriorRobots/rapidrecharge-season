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

  // flipped concoders
  public static final boolean TURRET_ENCODER_REVERSED = false;

  // turret
  public static final double MAX_ROTATION = 175; // degrees clockwise
  public static final double MIN_ROTATION = -175; // degrees clockwise
  public static final double TURRET_TOLERANCE = 2; // degrees
  public static final double TURRET_KP = 10;

  //TODO change values
  //shooter
  public static final double SHOOTER_DEFAULT = 0; //rpm
  public static final double SHOOTER_TOLERANCE = 200; // +rpm
  public static final double SHOOTER_KP = 0.15;
  public static final double SHOOTER_PRE = -0.2; // percent (for hopper and feed to pulse back)
  public static final double SHOOTER_FEED = 1; // percent (for hopper and feed to feed)
  public static final double SHOOTER_SLOW_FEED = 0.2; // percent (for hopper and feed to feed when the shooter is not rev-ed)
  public static final double SHOOTER_INTAKE_AGITATE = 0.25; // percent (to agitate balls with the intake)
}
