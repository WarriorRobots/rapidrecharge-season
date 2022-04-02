// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class FeedSubsystem extends SubsystemBase {
  /** Creates a new FeedSubsystem. */
  private WPI_TalonSRX m_feed;
  private DigitalInput m_feedInfraredSensor, m_intakeInfraredSensor;
  

  public FeedSubsystem() {
    m_feed = new WPI_TalonSRX(RobotMap.ID_FEED);
    m_feed.setInverted(Vars.FEED_REVERSED);
   m_feedInfraredSensor = new DigitalInput(RobotMap.ID_FEED_INFRARED);
   m_intakeInfraredSensor = new DigitalInput(RobotMap.ID_INTAKE_INFRARED);
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
  public boolean FeedcontainsBall()
  {
    return !m_feedInfraredSensor.get(); // infrared reads false when it sees a ball
    
  }
  public boolean IntakecontainsBall()
  {
    return !m_intakeInfraredSensor.get(); // infrared reads false when it sees a ball
  }
  /**
   * 
   * @return true, If 2 Balls Present in the Robot   */
  public boolean TwoBallsPresent() {
   return (FeedcontainsBall() && IntakecontainsBall());
    
  }
  /**
   * 
   * @return true, If one ball is present
   */
  public boolean oneBallPresent(){
    return (FeedcontainsBall() ^ IntakecontainsBall());
  }
  /**
   * 
   * @return true, if a ball is Present in the robot
   */
  public boolean BallPresent(){
    return (FeedcontainsBall() || IntakecontainsBall());
  }
/**
 * 
 * @return True, if no balls present
 */
  public boolean NoBallsPresent(){
    return !BallPresent();
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
