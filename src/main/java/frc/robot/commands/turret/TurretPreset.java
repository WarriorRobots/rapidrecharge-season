package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPreset extends CommandBase {
    /** Creates a new TurretPreset. */  
    private TurretSubsystem m_turret;
    private double m_target;
  
    /**
     * Rotates the target to the specified target rotation.
     * 
     * @param target The target the turret is to rotate to in degrees.
     */
    public TurretPreset(TurretSubsystem turret, double target) {
      m_turret = turret;
      this.m_target = target;
      addRequirements(this.m_turret);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
      m_turret.rotateToPosition(m_target);
    }
    
    @Override
    public boolean isFinished() {
      return Math.abs(m_target-m_turret.getRotationDegrees())<Vars.TURRET_TOLERANCE;
    }
  }
