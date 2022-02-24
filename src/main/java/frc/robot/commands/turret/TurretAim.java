// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAim extends CommandBase {
  /** Creates a new TurretAim. */
  private CameraSubsystem m_Camera;
  private TurretSubsystem m_Turret;

  public TurretAim(CameraSubsystem Camera,TurretSubsystem Turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Camera = Camera;
    m_Turret = Turret;
    addRequirements(Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Camera.ChangePipeline(CameraSubsystem.Tracking_Pipline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Camera.TargetExists()) {
      m_Turret.rotateBounded(m_Turret.getRotationDegrees() + m_Camera.GetTargetX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Camera.ChangePipeline(CameraSubsystem.Drive_Pipline);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Camera.TargetExists() && Math.abs(m_Camera.GetTargetX())<Vars.TURRET_TOLERANCE;
  }
}
