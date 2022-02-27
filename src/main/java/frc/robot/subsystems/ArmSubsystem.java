// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  
  private WPI_TalonSRX m_arm;
  private DigitalInput m_hallEffect;

  public ArmSubsystem() {
    m_arm = new WPI_TalonSRX(RobotMap.ID_ARM);
    m_arm.setInverted(Vars.ARM_REVERSED);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    // m_arm.setSensorPhase(Vars.ARM_ENCODER_REVERSED);
    // m_arm.config_kP(Constants.PRIMARY_PID, Vars.ARM_P, Constants.MS_TIMEOUT);
    m_hallEffect = new DigitalInput(RobotMap.ID_ARM_HALLEFFECT);
  }

  /**
   * Gives a percentage to the Arm motor.
   * <p><b>This method does not have a safety! This means the arm could crash by continuing to run and hit the frame!</b>
   * @param percent Percent from -1.0 to 1.0. (1.0 extends the arm out of the bot, -1.0 retracts the arm into the bot)
   */
  public void setPercentage(double percent)
  {
    m_arm.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Commands the arm to a position.
   * <p><b>This method does not have a safety! This means the arm could crash by commanding the wrong position!</b>
   * @param degrees Position to rotate to in degrees.
   */
  public void setAngleUnbounded(double degrees)
  {
    m_arm.set(ControlMode.Position, toNative(degrees));
  }

  /**
   * Commands the arm to a position.
   * @param degrees Position to rotate to in degrees.
   */
  public void setAngleBounded(double degrees)
  {
    //TODO change min/max angles
    if (degrees < Vars.ARM_MINIMUM_ANGLE) {
      m_arm.set(ControlMode.Position, toNative(Vars.ARM_MINIMUM_ANGLE));
      System.out.println("Arm moving to " + degrees + ", cutting short to prevent crash!");
    } else if (degrees > Vars.ARM_MINIMUM_ANGLE) {
      m_arm.set(ControlMode.Position, toNative(Vars.ARM_MAXIMUM_ANGLE));
      System.out.println("Arm moving to " + degrees + ", cutting short to prevent crash!");
    } else {
      m_arm.set(ControlMode.Position, toNative(degrees));
    }  }

  /** 
   * Return the encoder value of the arm.
   * @return native units
   */
  public double getEnc()
  {
    return m_arm.getSelectedSensorPosition();
  }

  /**
   * Return the position of the arm.
   * @return degrees
   */
  public double getPosition()
  {
    return toDegrees(m_arm.getSelectedSensorPosition());
  }

  /**
   * Convert from native units to degrees
   * @param native_units native units
   * @return degrees
   */
  public double toDegrees(double native_units)
  {
    return (double) native_units / Constants.CLICKS_PER_REV_QUADRATURE * 360.0;
  }

  /**
   * Convert from degrees to native units.
   * @param degrees degrees
   * @return native units
   */
  public double toNative(double degrees)
  {
    return (int) Math.round(degrees * Constants.CLICKS_PER_REV_QUADRATURE / 360.0);
  }

  /**
   * @return True when the arm is mechanically zeroed.
   */
  public boolean getHallEffect()
  {
    return !m_hallEffect.get(); // hall effect sensor reads false when the arm is at it's physical zero
  }

  /**
   * Reset the arm encoder to 0.
   */
  public void reset()
  {
    m_arm.setSelectedSensorPosition(0);
  }

  /**
   * Sets the arm encoder to a value in degrees.
   * @param degrees The angle the arm is mechanically at this method is called.
   */
  public void setDegrees(double degrees)
  {
    m_arm.setSelectedSensorPosition(toNative(degrees));
  }

  /**
   * Stops the arm.
   */
  public void stop()
  {
    m_arm.stopMotor();
  }

  @Override
  public void periodic()
  {

    // This method will be called once per scheduler run
    if (getHallEffect()) {
      reset();
    }

  }
}
