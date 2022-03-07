// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMakeRoom extends CommandBase {
  private ArmSubsystem m_arm;

  /** 
   * This command moves the arm away from the shooter if it is near the shooter.
   */
  public ArmMakeRoom(ArmSubsystem arm) {
    addRequirements(arm);
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // when the arm is closer than the ARM_IN position, it is too close to the shooter
    if (m_arm.getPosition() < Vars.ARM_IN) m_arm.setAngleBounded(Vars.ARM_IN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // check if the arm position is past the negative tolerance of the arm in
    return m_arm.getPosition() > Vars.ARM_IN - Vars.ARM_TOLERANCE;
  }
}
