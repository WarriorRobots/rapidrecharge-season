// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimANDShoota extends CommandBase {
  ShooterSubsystem m_shooter;
  TurretSubsystem m_turret;
  CameraSubsystem m_camera;

  /** Creates a new AimANDShoota. */
  public AimANDShoota(ShooterSubsystem shooter, TurretSubsystem turret, CameraSubsystem camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    addRequirements(m_shooter);
    m_turret = turret;
    addRequirements(m_turret);
    m_camera = camera;
    addRequirements(m_camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // IBRAHIM CODE IBRAHIM CODE

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If target exists
    if(m_camera.TargetExists())
    {
      // rotate turret to object with x value
      m_turret.rotateBounded(m_turret.getRotationDegrees() + m_camera.GetTargetX());
      if(Math.abs(m_camera.GetTargetX())<= Vars.TURRET_TOLERANCE){
        //m_shooter.getCurrentCommand();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
