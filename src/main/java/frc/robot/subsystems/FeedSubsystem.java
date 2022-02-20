// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeedSubsystem extends SubsystemBase {
  /** Creates a new FeedSubsystem. */
  public FeedSubsystem() {
    // TODO Feed subsystem has 1 WPI_TalonSRX
    // TODO Feed has an infrared sensor; see: https://github.com/WarriorRobots/infiniterecharge-season/blob/d32cc45dbcdcf51c9a6d12789ab94d90367efd9a/src/main/java/frc/robot/subsystems/FeedSubsystem.java#L32
  }

  /**
   * Run the Feed at a percentage.
   * @param percent Percent -1.0 to 1.0
   */
  public void setPercentage(double percent)
  {
    // TODO set percentage of the feed
  }

  /**
   * @return a boolean whether a ball is present in the Feed
   */
  public boolean containsBall()
  {
    return false; // TODO
  }

  /**
   * Stops the feed motor.
   */
  public void stop()
  {
    // TODO
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
