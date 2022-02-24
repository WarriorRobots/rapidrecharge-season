// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HopperSubsystem extends SubsystemBase {
  /** Creates a new HopperSubsystem. */

  private WPI_TalonSRX m_belt_top;
  private WPI_TalonSRX m_belt_bottom;

  public HopperSubsystem() {
    // TODO Change IDs
    m_belt_top = new WPI_TalonSRX(RobotMap.ID_BELT_TOP);
    m_belt_bottom = new WPI_TalonSRX(RobotMap.ID_BELT_BOTTOM);
  }

  /**
   * Run the Hopper at a percentage.
   * @param top Percent -1.0 to 1.0
   * @param bottom Percent -1.0 to 1.0
   */
  public void setPercentage(double top, double bottom)
  {
    m_belt_top.set(ControlMode.PercentOutput, top);
    m_belt_bottom.set(ControlMode.PercentOutput, bottom);
  }

  /**
   * Stops the Hopper motors.
   */
  public void stop()
  {
    m_belt_top.stopMotor();
    m_belt_bottom.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
