// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFeed extends CommandBase {
  /** Creates a new ShooterFeed. */
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  FeedSubsystem m_feed;
  DoubleSupplier m_ShooterRPM, m_SHOOTERRPMBACK;
  public ShooterFeed(ShooterSubsystem shooter, IntakeSubsystem intake, FeedSubsystem feed, DoubleSupplier ShooterRPMFRONT, DoubleSupplier SHOOTERRPMBACK) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    addRequirements(m_shooter);
    m_intake = intake;
    addRequirements(m_intake);
    m_feed = feed;
    addRequirements(m_feed);
    m_ShooterRPM = ShooterRPMFRONT;
    m_SHOOTERRPMBACK = SHOOTERRPMBACK;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    // TODO this logic does not match that of AimShootFeed
    if(Math.abs(m_shooter.getRPMFront()-m_ShooterRPM.getAsDouble()) < Vars.SHOOTER_TOLERANCE){
      m_intake.setPercentage(Vars.SHOOTER_FEED, Vars.SHOOTER_FEED);
      m_feed.setPercentage(Vars.SHOOTER_FEED);
    }else{
      if(!m_feed.FeedcontainsBall()){
        m_intake.setPercentage(Vars.SHOOTER_SLOW_FEED, Vars.SHOOTER_SLOW_FEED);
        m_feed.setPercentage(Vars.SHOOTER_SLOW_FEED);
      }else{
        m_intake.stop();
        m_feed.stop();
      }
    }
    m_shooter.setRPM(m_ShooterRPM.getAsDouble(), m_SHOOTERRPMBACK.getAsDouble());
  }




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.stop();
    m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
