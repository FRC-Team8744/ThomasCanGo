// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistProfiled extends CommandBase {
  private final DriveSubsystem m_drive;
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(0.5, 0.3);
  ProfiledPIDController m_moveCtrl = new ProfiledPIDController(DriveConstants.kMoveP, DriveConstants.kMoveI, DriveConstants.kMoveD, m_constraints);
  private double m_output;
  private double m_distance;
  private double m_goalDistance;

  /** Creates a new DriveDistProfiled. */
  public DriveDistProfiled(double targetDistance, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);

    m_goalDistance = targetDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_moveCtrl.setTolerance(0.01);  // in meters!
    m_moveCtrl.setGoal(m_goalDistance);
    m_moveCtrl.reset(0.0);

    // Debug information
    SmartDashboard.putData("movePID", m_moveCtrl);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_distance = m_drive.getAverageEncoderDistance();
    m_output = MathUtil.clamp(m_moveCtrl.calculate(m_distance), -1.0, 1.0);
    // Send PID output to drivebase
    m_drive.arcadeDrive(m_output, 0.0, false);

    // Debug information
    SmartDashboard.putNumber("PID setpoint", m_goalDistance);
    SmartDashboard.putNumber("PID output", m_output);
    SmartDashboard.putNumber("PID setpoint error", m_moveCtrl.getPositionError());
    SmartDashboard.putNumber("PID velocity error", m_moveCtrl.getVelocityError());
    SmartDashboard.putNumber("PID measurement", m_distance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_moveCtrl.atSetpoint();
  }
}
