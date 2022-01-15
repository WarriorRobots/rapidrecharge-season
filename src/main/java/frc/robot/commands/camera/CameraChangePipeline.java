// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.camera;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CameraSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CameraChangePipeline extends InstantCommand {
  private CameraSubsystem m_CameraSub;
  int pipeline;
  public CameraChangePipeline(CameraSubsystem camera, int pipline) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CameraSub = camera;
    addRequirements(this.m_CameraSub);
    this.pipeline = pipline;
  }

  // Called when the command is initially scheduled.
  @Override


  public void initialize() {
    m_CameraSub.ChangePipeline(pipeline);
  }
}
