/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.RobotMap;
import frc.robot.Vars;

/**
 * Contains the a single wheel shooter with a PID to command the velocity of the shooter.
 */
public class ShooterSubsystem extends SubsystemBase {
  
  private WPI_TalonFX m_shooter_left;
  private WPI_TalonFX m_slave_right;
  private WPI_TalonSRX m_back_motor;
  
  // TODO the below variables should be sparated into one for the front and one for the back motor

  /**
   * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final ShooterSubsystem shooter = new
	 * ShooterSubsystem();
	 */
  public ShooterSubsystem()
  {
    //TODO change sensors?
    m_shooter_left = new WPI_TalonFX(RobotMap.ID_SHOOTER_LEFT);
    m_shooter_left.setInverted(Vars.SHOOTER_LEFT_REVERSED);
    m_shooter_left.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT); //XXX problem with this line:w
    m_shooter_left.config_kF(Constants.PRIMARY_PID, Vars.SHOOTER_FRONT_ESTIMATED_PERCENTAGE*1023/Vars.SHOOTER_FRONT_NATIVE_ESTIMATED_VELOCITY, Constants.MS_TIMEOUT); // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
    m_shooter_left.config_kP(Constants.PRIMARY_PID, Vars.SHOOTER_FRONT_KP, Constants.MS_TIMEOUT);
    
    m_slave_right = new WPI_TalonFX(RobotMap.ID_SHOOTER_RIGHT);
    m_slave_right.follow(m_shooter_left);
    m_slave_right.setInverted(Vars.SHOOTER_RIGHT_REVERSED);
  
    m_back_motor = new WPI_TalonSRX(RobotMap.ID_SHOOTER_KICKER);
    m_back_motor.setInverted(Vars.SHOOTER_BACK_INVERTED);
    m_back_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    m_back_motor.config_kF(Constants.PRIMARY_PID, Vars.SHOOTER_BACK_ESTIMATED_PERCENTAGE*1023/Vars.SHOOTER_BACK_NATIVE_ESTIMATED_VELOCITY, Constants.MS_TIMEOUT);
    m_back_motor.config_kP(Constants.PRIMARY_PID, Vars.SHOOTER_BACKSPIN_KP, Constants.MS_TIMEOUT); // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
  }


  /**
   * Run the shooter motor at a percent from -1 to 1.
   * @param front Percent from -1 to 1.
   * @param back Percent from -1 to 1.
   */
  public void setPercentage(double front, double back)
  {
    m_shooter_left.set(ControlMode.PercentOutput, front);
    m_back_motor.set(ControlMode.PercentOutput, back);
  }

  /**
   * Run the shooter motor at an RPM.
   * @param front RPM of the shooter flywheel. (Not the motor or encoder.)
   * @param back RPM of the shooter back wheel. (Not the motor or encoder.)
   */
  public void setRPM(double front, double back)
  {
    m_shooter_left.set(ControlMode.Velocity, toNative_Front(front));
    m_back_motor.set(ControlMode.Velocity, toNative_Back(back));
  }

  /**
   * Get the percent the talon is commanding to the front motors.
   * @return Percentage from -1 to 1.
   */
  public double getGainFront()
  {
    return m_shooter_left.getMotorOutputPercent();
  }

  /**
   * Get the percent the talon is commanding to the back motor.
   * @return Percentage from -1 to 1.
   */
  public double getGainBack()
  {
    return m_back_motor.getMotorOutputPercent();
  }


  /**
   * @return the RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public double getRPMFront()
  {
    // (native / 100ms) * (600ms / m) * (rev/native) = rev / m
    return toRPM_Front(m_shooter_left.getSelectedSensorVelocity());
  }

  /**
   * @return the RPM of the shooter back wheel. (Not the motor or encoder.)
   */
  public double getRPMBack()
  {
    return toRPM_Back(m_back_motor.getSelectedSensorVelocity());
  }

  /**
   * Get the raw encoder value of the front motor
   * @return native units
   */
  public double getEncFront()
  {
    return m_shooter_left.getSelectedSensorPosition();
  }

  /**
   * Get the raw velocity of the front motor
   * @return Native units / 100ms
   */
  public double getEncVelocityFront()
  {
    return m_shooter_left.getSelectedSensorVelocity();
  }

  /**
   * Converts from native units per 100ms to RPM for the front wheel.
   * @param native_units Native units / 100ms
   * @return RPM
   */
  public static double toRPM_Front(double native_units)
  { 
    return ((native_units * 600 * Vars.SHOOTER_FRONT_GEARING) / Constants.CLICKS_PER_REV_INTEGRATED);
  }

  /**
   * Converts from native units per 100ms to RPM for the front wheel.
   * @param native_units Native units / 100ms
   * @return RPM
   */
  public static double toRPM_Back(double native_units)
  {
    return ((native_units * 600) / Constants.CLICKS_PER_REV_QUADRATURE);
  }
  
  /**
   * Converts from RPM to native units per 100ms.
   * @param rpm RPM
   * @return Native units / 100ms
   */
  public static double toNative_Front(double rpm)
  { 
    return ((rpm / 600 / Vars.SHOOTER_FRONT_GEARING) * Constants.CLICKS_PER_REV_INTEGRATED);
  }

  /**
   * Converts from RPM to native units per 100ms.
   * @param rpm RPM
   * @return Native units / 100ms
   */
  public static double toNative_Back(double rpm)
  {
    return ((rpm / 600) * Constants.CLICKS_PER_REV_QUADRATURE);
  }


  public void stop()
  {
    m_shooter_left.stopMotor();
    m_back_motor.stopMotor();
  }

  @Override
  public void periodic()
  {
  }

}