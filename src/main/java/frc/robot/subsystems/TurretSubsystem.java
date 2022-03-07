package frc.robot.subsystems;

import javax.management.ConstructorParameters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class TurretSubsystem extends SubsystemBase {
  private WPI_TalonSRX turret;   


  public TurretSubsystem () {
    turret = new WPI_TalonSRX(RobotMap.ID_TURRET);
    turret.setInverted(Vars.TURRET_REVERSED);
    turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
	turret.setSensorPhase(Vars.TURRET_ENCODER_REVERSED);
    turret.config_kP(Constants.PRIMARY_PID, Vars.TURRET_KP, Constants.MS_TIMEOUT);
  }  

  /** 
   * Give the turret a voltage to rotate.
   * WARNING for testing, should have safety for normal use
   * 
   * @param voltage Decimal percentage from -1 to 1. 1 is clockwise.
   */
  public void rotateNoSafety(double voltage) {
    turret.set(ControlMode.PercentOutput, voltage);
  }

  /**
   * Give the turret a voltage to rotate.
   * @param percent Decimal percentage from -1 to 1. 1 is clockwise.
   */
  public void rotateSafety(double percent)
  {
    double position = getRotationDegrees();
    if (position<Vars.TURRET_MIN_ROTATION) // counterclockwise bound
    {
      turret.set(ControlMode.PercentOutput, (percent<0) ? 0 : percent);
    }
    else if (position>Vars.TURRET_MAX_ROTATION) // clockwise bound
    {
      turret.set(ControlMode.PercentOutput, (0<percent) ? 0 : percent);
    }
    else
    {
      turret.set(ControlMode.PercentOutput, percent);
    }
  }

  /**
   * Rotates to a position given as a heading relative to the robot
   * 
   * @param position Double representing where the robot is rotated in degrees
   */  
  public void rotateToPosition(double position) {
    if (position<Vars.TURRET_MIN_ROTATION) {
      turret.set(ControlMode.Position, toClicks(Vars.TURRET_MIN_ROTATION));
    }
    else if (position>Vars.TURRET_MAX_ROTATION) {
      turret.set(ControlMode.Position, toClicks(Vars.TURRET_MAX_ROTATION));
    }
    else {
      turret.set(ControlMode.Position, toClicks(position));
    }
  }

  
  /**
   * Rotates the turret to the point closest to the position while being constrained to to the min and max degrees.
   * If it needs to, it will rotate the other direction to get to it's commanded position.
   */  
  public void rotateBounded(double position){
    // if the turret is trying to rotate over it's min or max, it should rotate around the other direction
    // this is done by knowing the current count of full rotations and perserving that but bounding the position
    if (position < Vars.TURRET_MIN_ROTATION || position > Vars.TURRET_MAX_ROTATION) {
      int count = (int) ( position/360 ); // count of rotations made
      position = bound(position) + 360 * count;
    }
    rotateToPosition(position);
  }
  /**
   * 
   * @return The Gain of The turret Motor
   */
  public double getTurretGain(){
    return turret.getMotorOutputPercent();
  }
  /**
   * Stops the turret motor.
   */  
  public void stop() {
    turret.stopMotor();
  }
  
  
  /** 
   * Gets the rotation of the turret based on encoder value
   * 
   * @return Degree rotation of turret. (+degree is clockwise)
   */
  public double getRotationDegrees() {
    return turret.getSelectedSensorPosition() / Constants.CLICKS_PER_REV_QUADRATURE * 360.0;
  }

  /** 
   * Converts betweens degrees and encoder clicks.
   */  
  public int toClicks(double degrees) {
    return (int) Math.round(degrees * Constants.CLICKS_PER_REV_QUADRATURE / 360.0);
  }

  public double getClicks(){
    return turret.getSelectedSensorPosition();
  }

  /** 
   * Converts betweens encoder clicks and degrees.
   */  
  public double toDegrees(double clicks) {
    return clicks/Constants.CLICKS_PER_REV_QUADRATURE * 360.0;
  }

  /**
   * Takes an angle and bounds it between -180 and 180. (So the angle can only be a half revolution from 0.)
   * @return Same angle within -180 inclusive to 180 exclusive.
   */  
  public double bound(double degrees) {
    if (degrees>=0) {
      return (degrees+179)%360-179;
    }
    else {
      return -((-degrees+179)%360-179);
    }
  } 

}
