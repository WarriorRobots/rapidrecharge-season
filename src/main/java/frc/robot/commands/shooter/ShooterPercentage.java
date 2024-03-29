/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPercentage extends CommandBase {
  private ShooterSubsystem m_shooter;
  private DoubleSupplier m_front_percent, m_back_percent;

  /**
   * Creates a new ShooterVoltage.
   */
  public ShooterPercentage(ShooterSubsystem shooter, DoubleSupplier front_percent, DoubleSupplier back_percent) {
    m_shooter = shooter;
    m_front_percent = front_percent;
    m_back_percent = back_percent;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setPercentage(m_front_percent.getAsDouble(), m_back_percent.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
