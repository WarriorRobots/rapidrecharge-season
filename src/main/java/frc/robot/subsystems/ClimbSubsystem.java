// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private WPI_TalonFX m_extension;
  private DoubleSolenoid m_angle;
  public ClimbSubsystem() {
    m_extension = new WPI_TalonFX(RobotMap.ID_CLIMB_MOTOR);
    m_extension.setInverted(Vars.CLIMB_MOTOR_REVERSED);
    m_extension.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    m_extension.config_kP(Constants.PRIMARY_PID, Vars.CLIMB_KP, Constants.MS_TIMEOUT);
    m_extension.config_kI(Constants.PRIMARY_PID, Vars.CLIMB_KI, Constants.MS_TIMEOUT);
    m_extension.config_kD(Constants.PRIMARY_PID, Vars.CLIMB_KD, Constants.MS_TIMEOUT);
    m_angle = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.ID_EXTENSION, RobotMap.ID_RECALL);
  }

  /**
   * Linearly moves climb WITHOUT SAFETY
   * @param percent -1.0 to 1.0
   */
  public void LinearClimb(double percent){
    m_extension.set(ControlMode.PercentOutput, percent);
  }
  /**
   * Give a position for the climb to go to.
   * @param position Desired position of the physical climb in inches upwards.
   */
  public void ClimbExtend(double position) {
    if (position < Vars.CLIMB_MINIMUM){
      m_extension.set(ControlMode.Position, toClicks(Vars.CLIMB_MINIMUM));
    }
    else if (position > Vars.CLIMB_MAXIMUM) {
      m_extension.set(ControlMode.Position, toClicks(Vars.CLIMB_MAXIMUM));
    } 
    else {
      m_extension.set(ControlMode.Position, toClicks(position));
    }
  }

  /**
   * Get the encoder of the winch.
   * @return Number of clicks on the encoder
   */
  public double getEnc() {
    return m_extension.getSelectedSensorPosition();
  }

  /**
   * Get the position of the physical climb.
   * @return Displacement of the climb upwards in inches.
   */
  public double getPosition() {
    return toInches(m_extension.getSelectedSensorPosition());
  }

  /**
   * Convert inches of movement of the climb into clicks of movement of the motor.
   * @param inches Number of inches of upwards motion of the climb.
   * @return Number of clicks of rotation of the motor.
   */
  public double toClicks(double inches) {
    return (inches / Math.PI / Vars.CLIMB_TRACK_DIAMETER / Vars.CLIMB_GEARING * Vars.CLIMB_CLICKS_PER_REV);
  }
/**
   * Convert clicks of movement of the motor into inches of movement of the climb.
   * @param clicks Number of clicks of rotation of the motor.
   * @return Number of Clicks of rotation of the motor.
   */
  public double toInches(double clicks) {
    return clicks / Vars.CLIMB_CLICKS_PER_REV * Vars.CLIMB_GEARING * Math.PI * Vars.CLIMB_TRACK_DIAMETER;
  }

  /**
   * Set the angle of climb. <p>
   * @param value A {@link DoubleSolenoid.Value}
   */
  public void setAngle(ClimbState state){
    m_angle.set(state.getValue());
  }
  /**
   * kOff -> None/stop <p>
   * kReverse -> Arm down <p>
   * kForward -> Arm Up <p>
   */
  public static enum ClimbState {
    stop(DoubleSolenoid.Value.kOff),
    armdown(DoubleSolenoid.Value.kReverse),
    armup(DoubleSolenoid.Value.kForward);
    public DoubleSolenoid.Value value;
    ClimbState(DoubleSolenoid.Value value) {
      this.value = value;
    }
    public DoubleSolenoid.Value getValue() {
      return value;
    }
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
