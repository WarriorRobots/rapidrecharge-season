// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;


//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vars;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;


public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private WPI_TalonFX m_frontLeft, m_rearLeft, m_frontRight, m_rearRight;
  private MotorControllerGroup m_left, m_right;
  private DifferentialDrive m_drive;
  private AHRS navx;
  private DifferentialDriveOdometry odometry;
  //private AHRS navx;
  
 
  /** Creates a new DriveTrain. */
  public DrivetrainSubsystem() {
    m_frontLeft = new WPI_TalonFX(RobotMap.ID_LEFT_FRONT);
    m_frontLeft.configOpenloopRamp(Vars.DRIVETRAIN_RAMP_TIME);
    m_frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    m_rearLeft = new WPI_TalonFX(RobotMap.ID_LEFT_REAR);
    m_rearLeft.configOpenloopRamp(Vars.DRIVETRAIN_RAMP_TIME);

    m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
 
    m_frontRight = new WPI_TalonFX(RobotMap.ID_RIGHT_FRONT);
    m_frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    m_frontRight.configOpenloopRamp(Vars.DRIVETRAIN_RAMP_TIME);
    m_rearRight = new WPI_TalonFX(RobotMap.ID_RIGHT_REAR);
    m_rearRight.configOpenloopRamp(Vars.DRIVETRAIN_RAMP_TIME);
    m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
 
    m_left.setInverted(Vars.LEFT_DRIVE_INVERTED);
    m_right.setInverted(Vars.RIGHT_DRIVE_INVERTED);

    m_drive = new DifferentialDrive(m_left, m_right);
    // navx = new AHRS(I2C.Port.kMXP);
     // There is no try / catch because if there is no navx on the robot, the auto would break anyways
     navx = new AHRS(I2C.Port.kMXP);
     // Creates odometry class with an initial angle of the current heading of the robot (which should be 0)
     odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngleDegrees()));
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }
  public void arcadedriveRaw(double x, double z) {
    m_drive.arcadeDrive(x, z, false);
  }
   /**
   * @return The inches/second of the left encoder.
   */
  public double getLeftVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) m_frontLeft.getSelectedSensorVelocity()  * 10 / Constants.CLICKS_PER_REV_INTEGRATED * Vars.DRIVETRAIN_GEARING * Math.PI * Vars.WHEEL_DIAMETER;
  }
  
  /**
   * @return The inches/second of the right encoder.
   */
  public double getRightVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) m_frontRight.getSelectedSensorVelocity()  * 10 / Constants.CLICKS_PER_REV_INTEGRATED* Vars.DRIVETRAIN_GEARING * Math.PI * Vars.WHEEL_DIAMETER;
  }
  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings in inches
   */
  public double getAveragePosition() {
    return (getLeftPosition() + getRightPosition()) / 2.0;
  }


  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return Wheel speeds (in meters/s)
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      Units.inchesToMeters(getLeftVelocity()),
      Units.inchesToMeters(getRightVelocity())
    );
  }

   /**
   * Drive the tankdrive with raw voltage values to give to the motors.
   * @param leftVoltage voltage given to the left side in Volts
   * @param rightVoltage voltage given to the right side in Volts
   */
  public void tankdriveVoltage(double leftVoltage, double rightVoltage) {
    // -rightVoltage to make the right side act reversed
    m_left.setVoltage(leftVoltage);
    m_right.setVoltage(rightVoltage);
    m_drive.feed();
  }
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  /**
   * Return estimated pose of the robot.
   * 
   * @return current pose (in meters)
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  /**
   * Resets the odometry of the robot
   * @param pose of the robot when it is reset (can be a default pose to )
   */
  public void resetOdometry() {
    reset();
    odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getAngleDegrees()));
  }
  /**
   * Zeros the left drivetrain encoder.
   */
  public void resetLeftEnc() {
    m_frontLeft.setSelectedSensorPosition(0);
  }
  /**
   * Zeros the right drivetrain encoder.
   */
  public void resetRightEnc() {
    m_frontRight.setSelectedSensorPosition(0);
  }
  /**
   * Zeros the drivetrain encoders.
   */
  public void resetEnc() {
    resetLeftEnc();
    resetRightEnc();
  }
   /**
   * Zeros ALL the sensors affiliated with the drivetrain.
   */
  public void reset() {
    resetAngle();
    resetEnc();
  }
  /**
   * Zeros the drivetrain yaw angle.
   */
  public void resetAngle() {
    navx.zeroYaw();
  }
  public void stop(){
    m_drive.stopMotor();
  }
  /**
	 * Gets yaw angle of the robot. (+Clockwise)
   * Applications that require counterclockwise as postive should take the neagtive of this value.
   * @return angle in degrees.
	 */
  public double getAngleDegrees() {
    return navx.getAngle();
}

 /**
  * @return The inches position of the left encoder.
  */
 public double getLeftPosition() {
   // clicks * rev/clicks * output/input = revs
   // revs * PI * diameter = distance
   return (double) m_frontLeft.getSelectedSensorPosition() * -1 / Constants.CLICKS_PER_REV_INTEGRATED * Vars.DRIVETRAIN_GEARING * Math.PI * Vars.WHEEL_DIAMETER;
   // This is a * -1 because the motor is commanded to go backwards by the differential drive
   // so the motor is still backwards even though we give the differential drive a positive command
 }

 /**
  * @return The inches position of the right encoder.
  */
 public double getRightPosition() {
   // clicks * rev/clicks * output/input = revs
   // revs * PI * diameter = distance
   return (double) m_frontRight.getSelectedSensorPosition()  / Constants.CLICKS_PER_REV_INTEGRATED * Vars.DRIVETRAIN_GEARING* Math.PI * Vars.WHEEL_DIAMETER;
 }
 /**
  * Returns the heading of the robot.
  *
  * @return the robot's heading in degrees, from -180 to 180
  */
 public double getHeading() {
   return navx.getRotation2d().getDegrees();
 }
 /** Zeroes the heading of the robot. */
 public void zeroHeading() {
   navx.reset();
 }
  /**
  * Returns the turn rate of the robot.
  *
  * @return The turn rate of the robot, in degrees per second
  */

 public double getTurnRate() {
   return -navx.getRate();
 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      Rotation2d.fromDegrees(-getAngleDegrees()), // - degrees because the CCW must be + as it is on a cartesion plane
      Units.inchesToMeters(getLeftPosition()),
      Units.inchesToMeters(getRightPosition())
    );
  }
}
