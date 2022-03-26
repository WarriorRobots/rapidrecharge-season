// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vars;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  public ClimbSequence(ClimbSubsystem climb) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // extend the pistons(to angle arm)
      new ClimbPiston(climb, ClimbState.armup),
      // extend the arms via falcon
      new ClimbMagic(climb, Vars.CLIMB_UP),
      // retract the pistons
      new ClimbPiston(climb, ClimbState.armdown),
      // retract arm via falcon
      new ClimbMagic(climb, Vars.CLIMB_DOWN)
    );
  }
}
