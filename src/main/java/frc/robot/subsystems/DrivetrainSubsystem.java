// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.I2C;


//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Vars;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class DrivetrainSubsystem extends SubsystemBase {
  WPI_VictorSPX m_frontLeft, m_rearLeft, m_frontRight, m_rearRight;
  MotorControllerGroup m_left, m_right;
  DifferentialDrive m_drive;
  private AHRS navx;
  
  /** Creates a new DriveTrain. */
  public DrivetrainSubsystem() {
    m_frontLeft = new WPI_VictorSPX(RobotMap.ID_FRONTLEFT);
    m_rearLeft = new WPI_VictorSPX(RobotMap.ID_REARLEFT);
    m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
 
    m_frontRight = new WPI_VictorSPX(RobotMap.ID_FRONTRIGHT);
    m_rearRight = new WPI_VictorSPX(RobotMap.ID_REARRIGHT);
    m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
 
    m_left.setInverted(Vars.LEFT_DRIVE_INVERTED);
    m_right.setInverted(Vars.RIGHT_DRIVE_INVERTED);

    m_drive = new DifferentialDrive(m_left, m_right);
    navx = new AHRS(I2C.Port.kMXP);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }
  public void arcadedriveRaw(double x, double z) {
    m_drive.arcadeDrive(x, z, false);
  }

  public void stop(){
    m_drive.stopMotor();
  }
  public double getAngle() {
    return navx.getAngle();
	}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
