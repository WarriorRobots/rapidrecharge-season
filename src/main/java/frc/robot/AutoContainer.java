/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.DashboardContainer.TabsIndex;
// import frc.robot.commands.auto.AutoHarvest;
// import frc.robot.commands.auto.RamseteContainer;
// import frc.robot.commands.auto.trajectories.TLine;
import frc.robot.DashboardContainer.TabsIndex;
import frc.robot.commands.auto.AutoBack;
import frc.robot.commands.auto.AutoIntakeShoot;
import frc.robot.commands.auto.AutoShootBack;
import frc.robot.commands.auto.AutoShootBackIntake;
import frc.robot.commands.auto.AutoShootBackIntakeShoot;
import frc.robot.commands.auto.RamseteContainer;
import frc.robot.commands.auto.trajectories.TBack;
import frc.robot.commands.auto.trajectories.TLine;
/**
 * A singleton tool to handle the auto selection.
 */
public class AutoContainer {

  private ShuffleboardTab autoTab = DashboardContainer.getInstance().getTab(TabsIndex.kAuto);
  private SendableChooser<Command> chooser = new SendableChooser<Command>();

  private static AutoContainer instance = null;

  // This constructor is private because it is a singleton
  private AutoContainer() {}

  private void init() {
    
    chooser.addOption("None", new InstantCommand());
    // chooser.addOption("Forwards 48\"",
    //   new RamseteContainer(RobotContainer.m_drivetrain, new TLine(){public double getLengthIn() {return 48;}}).getCommandAndStop()
    // );
    // chooser.addOption("AutoHarvest", new AutoHarvest(
    //     RobotContainer.m_drivetrain,
    //     RobotContainer.m_shooter,
    //     RobotContainer.m_turret,
    //     RobotContainer.m_camera,
    //     RobotContainer.m_feed,
    //     RobotContainer.m_hopper,
    //     RobotContainer.m_arm,
    //     RobotContainer.m_intake
    // ));
    // Facing Away from the target moves back at 38 inches
    chooser.addOption("BackUP", new AutoBack(RobotContainer.m_drivetrain, RobotContainer.m_ShooterSubsystem, RobotContainer.m_TurretSubsystem, RobotContainer.m_CameraSubsystem, RobotContainer.m_IntakeSubsystem, RobotContainer.m_FeedSubsystem, RobotContainer.m_ArmSubsytem));
    // Facing Away the target shoots and moves back
    chooser.addOption("ShootBack", new AutoShootBack(RobotContainer.m_drivetrain, RobotContainer.m_ShooterSubsystem, RobotContainer.m_TurretSubsystem, RobotContainer.m_CameraSubsystem, RobotContainer.m_IntakeSubsystem, RobotContainer.m_FeedSubsystem, RobotContainer.m_ArmSubsytem));
    // Facing away from the target robot shoots moves back and intakes a ball
    chooser.addOption("ShootBackIntake", new AutoShootBackIntake(RobotContainer.m_drivetrain, RobotContainer.m_ShooterSubsystem, RobotContainer.m_TurretSubsystem, RobotContainer.m_CameraSubsystem, RobotContainer.m_IntakeSubsystem, RobotContainer.m_FeedSubsystem, RobotContainer.m_ArmSubsytem));
    // Facing away from the target robot shoots moves back intakes and shoots again
    chooser.addOption("ShootBackIntakeShoot", new AutoShootBackIntakeShoot(RobotContainer.m_drivetrain, RobotContainer.m_ShooterSubsystem, RobotContainer.m_TurretSubsystem, RobotContainer.m_CameraSubsystem, RobotContainer.m_IntakeSubsystem, RobotContainer.m_FeedSubsystem, RobotContainer.m_ArmSubsytem));
    chooser.addOption("IntakeShoot", new AutoIntakeShoot(RobotContainer.m_drivetrain, RobotContainer.m_ShooterSubsystem, RobotContainer.m_TurretSubsystem, RobotContainer.m_CameraSubsystem, RobotContainer.m_IntakeSubsystem, RobotContainer.m_FeedSubsystem, RobotContainer.m_ArmSubsytem));
     chooser.addOption("Forward 1 Foot",
      new RamseteContainer(RobotContainer.m_drivetrain, new TLine(){public double getLengthIn() {return 51;}}).getCommandAndStop()
    );

    autoTab.add("Auto Selector", chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3,0).withSize(2, 1);
  
  }

  /**
   * Gets the AutoContainer instance.
   */
  public static AutoContainer getInstance() {
    if (instance==null) {
      instance = new AutoContainer();
      instance.init();
    }
    return instance;
  }

  public Command getAutoCommand() {
    return chooser.getSelected();
  }

}
