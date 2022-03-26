package frc.robot;

/**
 * All device IDs belong here. (Eg. Motor controllers, sensors, pneumatics, etc.)
 */
public class RobotMap {
  // CAN
  public static final int ID_LEFT_REAR = 0;
  public static final int ID_LEFT_FRONT = 1;
  public static final int ID_SHOOTER_LEFT = 2;
  public static final int ID_SHOOTER_KICKER = 3;
  public static final int ID_ARM = 4;
  public static final int ID_PICKUP_LOWER = 10;
  // public static final int ID_06 = 6;
  // The Limelight isn't a device with an ID, if it did, it would be 07
  // public static final int ID_08 = 8;
  public static final int ID_CLIMB_MOTOR = 9;
  public static final int ID_PICKUP_UPPER = 5;
  public static final int ID_FEED = 11;
  public static final int ID_TURRET = 12;
  public static final int ID_SHOOTER_RIGHT = 13;
  public static final int ID_RIGHT_REAR = 14;
  public static final int ID_RIGHT_FRONT = 15;
  

  // Pneumatics
  public static final int ID_PCM = 0;
  // TODO it should be noted the left and the right are symetric so they cannot be distinguished
  public static final int ID_CLIMB_EXTENSION_LEFT = 4;
  public static final int ID_CLIMB_RECALL_LEFT = 2;
  public static final int ID_CLIMB_EXTENSION_RIGHT = 3;
  public static final int ID_CLIMB_RECALL_RIGHT = 5;

  // DIO
  public static final int ID_FEED_INFRARED = 1;
  public static final int ID_INTAKE_INFRARED = 0; // TODO change once actual sensor is put on the robot
  public static final int ID_ARM_HALLEFFECT = 0;
  
  public static final int LED_CONTROLLER_ID = 0; 
}
