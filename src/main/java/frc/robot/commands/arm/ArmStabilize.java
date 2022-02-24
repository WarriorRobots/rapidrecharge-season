// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmStabilize extends InstantCommand {
    /** Creates a new ArmStabilize. */
  private ArmSubsystem m_arm;

  public ArmStabilize(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngleUnbounded(m_arm.getPosition());
  }
}
