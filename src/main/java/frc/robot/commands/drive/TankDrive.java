// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private DrivetrainSubsystem m_driveTrain;
  private DoubleSupplier m_leftValue, m_rightValue;
  
  public TankDrive(DrivetrainSubsystem driveTrain, DoubleSupplier leftValue, DoubleSupplier rightValue) {
    addRequirements(driveTrain);
    //driveTrain = m_driveTrain;
    //leftValue = m_leftValue;
    //rightValue = m_rightValue;
    m_driveTrain = driveTrain;
    m_leftValue = leftValue;
    m_rightValue = rightValue;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.tankDrive(m_leftValue.getAsDouble(),m_rightValue.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
