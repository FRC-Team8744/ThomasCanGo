// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistProfiled extends CommandBase {
  private final DriveSubsystem m_drive;
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
    DriveConstants.kMaxSpeedMetersPerSecond,
    DriveConstants.kMaxAccelerationMetersPerSecondSquared);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile profile;

  private double m_timestep;
  private double m_goalDistance;

  /** Creates a new DriveDistProfiled. */
  // Derived from: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html#trapezoidal-motion-profiles-in-wpilib
  public DriveDistProfiled(double targetDistance, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);

    m_goalDistance = targetDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goal = new TrapezoidProfile.State(m_goalDistance, 0);
    m_timestep = 0;

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    profile = new TrapezoidProfile(m_constraints, m_goal, new TrapezoidProfile.State(m_drive.getAverageEncoderDistance(), 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_timestep += 0.02;  // 20ms between scheduler runs
    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(m_timestep);

    // Send setpoint to offboard controller PID
    m_drive.setDriveStates(m_setpoint, m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profile.isFinished(m_timestep);
  }
}
