// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ClimbSetTurret extends CommandBase {
  private TurretSubsystem m_turret;
  /** Creates a new ClimbSetTurret. */
  public ClimbSetTurret(TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Determine whether position is closer to 180 or 0
    m_turret.bound(m_turret.getRotationDegrees());
    if (Math.abs(m_turret.getRotationDegrees()) < 90) {
      m_turret.rotateBounded(0);
    }
    else {
      m_turret.rotateBounded(180);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
