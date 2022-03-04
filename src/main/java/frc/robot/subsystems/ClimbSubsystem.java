// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private WPI_TalonFX m_extension;
  private DoubleSolenoid m_angle;
  public ClimbSubsystem() {
    m_extension = new WPI_TalonFX(RobotMap.ID_CLIMB_MOTOR);
    m_extension.setInverted(Vars.CLIMB_MOTOR_REVERSED);
    
    m_angle = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.ID_EXTENSION, RobotMap.ID_RECALL);
  }

  /**
   * Give a position for the climb to go to.
   * @param position Desired position of the physical climb in inches upwards.
   */
  public void ClimbExtend(int position) {
    if (position < Vars.CLIMB_MINIMUM){
      m_extension.set(ControlMode.Position, toClicks(Vars.CLIMB_MINIMUM));
    }
    else if (position > Vars.CLIMB_MAXIMUM) {
      m_extension.set(ControlMode.Position, toClicks(Vars.CLIMB_MAXIMUM));
    } else {
      m_extension.set(ControlMode.Position, toClicks(position));
    }
  }
  private double toClicks(int climbMinimum) {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
