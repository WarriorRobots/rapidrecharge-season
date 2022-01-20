package frc.robot;

public class Vars {
  // flipped motors
  public static final boolean LEFT_DRIVE_INVERTED = false;
  public static final boolean RIGHT_DRIVE_INVERTED = true;
  public static final boolean TURRET_REVERSED = false;

  // flipped concoders
  public static final boolean TURRET_ENCODER_REVERSED = false;

  // turret
  public static final double MAX_ROTATION = 60; // degrees clockwise
  public static final double MIN_ROTATION = -240; // degrees clockwise
  public static final double TURRET_KP = 10;
}
