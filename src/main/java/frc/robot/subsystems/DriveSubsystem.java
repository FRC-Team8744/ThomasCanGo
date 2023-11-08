// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
// import frc.robot.ExampleSmartMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create CAN motor objects
  private CANSparkMax leftFrontSparkMax = new CANSparkMax(DriveConstants.kLeftFrontCAN, MotorType.kBrushless);
  private CANSparkMax leftRearSparkMax = new CANSparkMax(DriveConstants.kLeftRearCAN, MotorType.kBrushless);
  private CANSparkMax rightFrontSparkMax = new CANSparkMax(DriveConstants.kRightFrontCAN, MotorType.kBrushless);
  private CANSparkMax rightRearSparkMax = new CANSparkMax(DriveConstants.kRightRearCAN, MotorType.kBrushless);

  // Group the motors on each side
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(leftFrontSparkMax, leftRearSparkMax);

  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(rightFrontSparkMax, rightRearSparkMax);

  // The robot's drivetrain
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  RelativeEncoder m_leftEncoder = leftFrontSparkMax.getEncoder();
  RelativeEncoder m_rightEncoder = rightFrontSparkMax.getEncoder();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);

    resetEncoders();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("m_leftEncoder", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("m_rightEncoder", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("leftEncoder(Inch)", m_leftEncoder.getPosition()/DriveConstants.kConvertInchToMeter);
    SmartDashboard.putNumber("rightEncoder(Inch)", m_rightEncoder.getPosition()/DriveConstants.kConvertInchToMeter);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

   /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   * @param sqr are inputs squared?
   */
  public void arcadeDrive(double fwd, double rot, boolean sqr) {
    m_drive.arcadeDrive(fwd, rot, sqr);
  }

  /**
   * Returns the left encoder distance.
   *
   * @return the left encoder distance
   */
  public double getLeftEncoderDistance() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Returns the right encoder distance.
   *
   * @return the right encoder distance
   */
  public double getRightEncoderDistance() {
    return m_rightEncoder.getPosition();
  }

  /** Resets the drive encoders. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
}

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}
