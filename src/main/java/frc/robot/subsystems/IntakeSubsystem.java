// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Vars;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private WPI_TalonSRX m_belt_top;
  private WPI_TalonSRX m_belt_bottom;

  public IntakeSubsystem() {
    m_belt_top = new WPI_TalonSRX(RobotMap.ID_PICKUP_UPPER);
    m_belt_top.setInverted(Vars.INTAKE_TOP_REVERSED);
    m_belt_bottom = new WPI_TalonSRX(RobotMap.ID_PICKUP_LOWER);
    m_belt_bottom.setInverted(Vars.INTAKE_BOTTOM_REVERSED);
  }

  /**
   * Run the Intake at a percentage.
   * @param top Percent -1.0 to 1.0
   * @param bottom Percent -1.0 to 1.0
   */
  public void setPercentage(double top, double bottom)
  {
    m_belt_top.set(ControlMode.PercentOutput, top);
    m_belt_bottom.set(ControlMode.PercentOutput, bottom);
  }

  /**
   * Stops the Intake motors.
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
