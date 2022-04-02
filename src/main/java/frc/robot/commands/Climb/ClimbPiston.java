// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbState;

public class ClimbPiston extends CommandBase {
  private ClimbSubsystem m_climb;
  private ClimbState m_state;
  private int counter;
  /** Creates a new ClimbPiston. */
  public ClimbPiston(ClimbSubsystem climb, ClimbState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    m_climb = climb;
    m_state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.setAngle(m_state);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.setAngle(ClimbState.stop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (counter > Vars.PNEUMATIC_LOOP_COUNT);
  }
}
