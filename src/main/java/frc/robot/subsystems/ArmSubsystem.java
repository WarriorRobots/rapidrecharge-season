// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // TODO Arm subsystem has 1 WPI_TalonSRX 
    // TODO The Talon needs to be configured to have a feedback sensor and kP
    // TODO Arm has a Hall Effect Sensor; see: https://github.com/WarriorRobots/infiniterecharge-season/blob/d32cc45dbcdcf51c9a6d12789ab94d90367efd9a/src/main/java/frc/robot/subsystems/ArmSubsystem.java#L46
  }

  /**
   * Gives a percentage to the Arm motor.
   * <p><b>This method does not have a safety! This means the arm could crash by continuing to run and hit the frame!</b>
   * @param percent Percent from -1.0 to 1.0. (1.0 extends the arm out of the bot, -1.0 retracts the arm into the bot)
   */
  public void setPercentage(double percent)
  {
    // TODO
  }

  /**
   * Commands the arm to a position.
   * <p><b>This method does not have a safety! This means the arm could crash by commanding the wrong position!</b>
   * @param degrees Position to rotate to in degrees.
   */
  public void setAngleUnbounded(double degrees)
  {
    // TODO (tip: finish toClicks() first)
  }

  /**
   * Commands the arm to a position.
   * @param degrees Position to rotate to in degrees.
   */
  public void setAngleBounded(double degrees)
  {
    // TODO (tip: finish toClicks() first)
  }

  /** 
   * Return the encoder value of the arm.
   * @return native units
   */
  public double getEnc()
  {
    return 0; // TODO
  }

  /**
   * Return the position of the arm.
   * @return degrees
   */
  public double getPosition()
  {
    return 0; // TODO
  }

  /**
   * Convert from native units to degrees
   * @param native_units native units
   * @return degrees
   */
  public double toDegrees(double native_units)
  {
    return 0; // TODO
  }

  /**
   * Convert from degrees to native units.
   * @param degrees degrees
   * @return native units
   */
  public double toNative(double degrees)
  {
    return 0; // TODO
  }

  /**
   * @return True when the arm is mechanically zeroed.
   */
  public boolean getHallEffect()
  {
    return false; // TODO
  }

  /**
   * Reset the arm encoder to 0.
   */
  public void reset()
  {
    // TODO
  }

  /**
   * Sets the arm encoder to a value in degrees.
   * @param degrees The angle the arm is mechanically at this method is called.
   */
  public void setDegrees(double degrees)
  {
    // TODO (note: this should convert the degrees to native units and set the encoder similar to reset())
  }

  /**
   * Stops the arm.
   */
  public void stop()
  {
    // TODO
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    // TODO when the hall effect is triggered, reset the arm encoder
  }
}
