package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretUnsafeRotate extends CommandBase {
    
  TurretSubsystem m_turret;
  DoubleSupplier m_input;

  /**
   * Rotate the turret linearly by use a supplier.
   * 
   * @param input A supplier/lambda that gives a double from a joystick or other input.
   */
  public TurretUnsafeRotate(TurretSubsystem turret, DoubleSupplier input) {
      m_turret = turret;
      m_input = input;
      addRequirements(this.m_turret);
    }

  @Override
  public void execute() {
    m_turret.rotateNoSafety(m_input.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

}
