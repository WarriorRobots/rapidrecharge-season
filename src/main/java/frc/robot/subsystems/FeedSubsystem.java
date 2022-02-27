// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class FeedSubsystem extends SubsystemBase {
  /** Creates a new FeedSubsystem. */
  private WPI_TalonSRX m_feed;
  private DigitalInput m_infraredSensor;

  public FeedSubsystem() {
    m_feed = new WPI_TalonSRX(RobotMap.ID_FEED);
   //m_infraredSensor = new DigitalInput(RobotMap.ID_FEED_INFRARED);
  }

  /**
   * Run the Feed at a percentage.
   * @param percent Percent -1.0 to 1.0
   */
  public void setPercentage(double percent)
  {
    m_feed.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @return a boolean whether a ball is present in the Feed
   */
  public boolean containsBall()
  {
    //return !m_infraredSensor.get(); // infrared reads false when it sees a ball
    return false;
  }

  /**
   * Stops the feed motor.
   */
  public void stop()
  {
    m_feed.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
