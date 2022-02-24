// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Linear extends CommandBase {
  /** Creates a new AutoStraight. */
  private DrivetrainSubsystem m_drive;
  private PIDController pidAngle;



  /** Creates a new AutoStraight. */
  public Linear(DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
    
    pidAngle= new PIDController(Vars.LINEAR_ANGLE_P, 0, 0);
  }
  public void setAngularPid(double p, double i, double d){
    pidAngle.setPID(p, i, d);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidAngle.setSetpoint(m_drive.getAngle());
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_drive.arcadedriveRaw(.2, pidAngle.calculate(m_drive.getAngle()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
