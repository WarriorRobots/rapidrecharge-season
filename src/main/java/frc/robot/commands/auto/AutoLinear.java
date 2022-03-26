// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Vars;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLinear extends CommandBase {
    /** Creates a new AutoLinear. */

    private DrivetrainSubsystem m_drive;

    private double m_setpoint; // inches
    private double m_endPosition; // inches
    private ProfiledPIDController profileController;
    private PIDController pidAngle;

    /**
     * Create a new instance of {@link DriveAuto}.
     * 
     * @param drive      Drivetrain subsystem
     * @param m_setpoint How many inches to travel.
     */
    public AutoLinear(DrivetrainSubsystem drive, double set_point) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        addRequirements(m_drive);
        m_setpoint = set_point;
        // sets default pid values
        profileController = new ProfiledPIDController(Vars.AUTO_LINEAR_P, Vars.AUTO_LINEAR_I, Vars.AUTO_LINEAR_D,
                new TrapezoidProfile.Constraints(Vars.MAX_VELOCITY, Vars.MAX_ACCELERATION));
        pidAngle = new PIDController(
                Vars.AUTO_LINEAR_ANGLE_P,
                0,
                0);
        profileController.setTolerance(Vars.AUTO_LINEAR_TOLERANCE);

    }

    /**
     * Set the internal distance PID constants to new values
     * 
     * @param p P gain
     * @param i I gain
     * @param d D gain
     */
    public void setDistancePid(double p, double i, double d) {
        profileController.setPID(p, i, d);
    }

    /**
     * Set an internal distance PID tolerance to a new value
     * 
     * @param tolerance Tolerance away from setpoint to be considered in range (in
     *                  inches)
     */
    // public void setDistanceTolerance(double tolerance) {
    // pidDistance.setTolerance(tolerance);
    // }

    /**
     * Set the internal angular PID constants to new values
     * 
     * @param p P gain
     * @param i I gain
     * @param d D gain
     */
    public void setAngularPid(double p, double i, double d) {
        pidAngle.setPID(p, i, d);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double averageSpeed = Units.metersToInches(
                m_drive.getWheelSpeeds().leftMetersPerSecond + m_drive.getWheelSpeeds().rightMetersPerSecond) / 2;
        profileController.reset(m_drive.getAveragePosition(), averageSpeed);
        m_endPosition = m_drive.getAveragePosition() + m_setpoint;
        profileController.setGoal(m_endPosition);
        pidAngle.reset();
        pidAngle.setSetpoint(m_drive.getAngleDegrees());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = profileController.calculate(m_drive.getAveragePosition());
        System.out.println(x);
        m_drive.arcadedriveRaw(
                x,
                pidAngle.calculate(m_drive.getAngleDegrees()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return profileController.atSetpoint();
        // return false;

        double averageSpeed = Units.metersToInches(
                m_drive.getWheelSpeeds().leftMetersPerSecond + m_drive.getWheelSpeeds().rightMetersPerSecond) / 2;
        return Math.abs(m_endPosition - m_drive.getAveragePosition()) < Vars.AUTO_LINEAR_TOLERANCE &&
                Math.abs(averageSpeed) < Vars.AUTO_SPEED_TOLERANCE;
    }
}