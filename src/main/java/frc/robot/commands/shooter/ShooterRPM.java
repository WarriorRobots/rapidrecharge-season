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

public class ShooterRPM extends CommandBase {
  /**
   * Creates a new ShooterRPM.
   */
  ShooterSubsystem m_shooter;
  DoubleSupplier m_rpm;
  // TODO

  public ShooterRPM(ShooterSubsystem shooter, DoubleSupplier front_rpm, DoubleSupplier back_rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_rpm = front_rpm;
    // TODO
    addRequirements(this.m_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO
    // m_shooter.setRPM(m_rpm.getAsDouble());
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
