// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    // TODO Hopper Subsystem has two WPI_TalonSRX's (one for the top belts and one for the bottom)
  }

  /**
   * Run the Hopper at a percentage.
   * @param top Percent -1.0 to 1.0
   * @param bottom Percent -1.0 to 1.0
   */
  public void setPercentage(double top, double bottom)
  {
    // TODO
  }

  /**
   * Stops the Hopper motors.
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
