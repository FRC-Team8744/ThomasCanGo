// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  // The motors on the left side of the drive.
  // private final ExampleSmartMotorController m_leftLeader =
  //     new ExampleSmartMotorController(DriveConstants.kLeftMotor1Port);

  // private final ExampleSmartMotorController m_leftFollower =
  //     new ExampleSmartMotorController(DriveConstants.kLeftMotor2Port);
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(leftFrontSparkMax, leftRearSparkMax);

  // The motors on the right side of the drive.
  // private final ExampleSmartMotorController m_rightLeader =
  //     new ExampleSmartMotorController(DriveConstants.kRightMotor1Port);

  // private final ExampleSmartMotorController m_rightFollower =
  //     new ExampleSmartMotorController(DriveConstants.kRightMotor2Port);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(rightFrontSparkMax, rightRearSparkMax);

  // The feedforward controller.
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  RelativeEncoder m_leftEncoder = leftFrontSparkMax.getEncoder();
  RelativeEncoder m_rightEncoder = rightFrontSparkMax.getEncoder();

  private SparkMaxPIDController m_leftPidController;
  private SparkMaxPIDController m_rightPidController;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Clear all previous settings from other programs.
    leftFrontSparkMax.restoreFactoryDefaults();
    leftRearSparkMax.restoreFactoryDefaults();
    rightFrontSparkMax.restoreFactoryDefaults();
    rightRearSparkMax.restoreFactoryDefaults();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
    // rightFrontSparkMax.setInverted(true);

    // You might need to not do this depending on the specific motor controller
    // that you are using -- contact the respective vendor's documentation for
    // more details.
    // m_rightFollower.setInverted(true);

    // m_leftFollower.follow(m_leftLeader);
    // m_rightFollower.follow(m_rightLeader);
    // leftRearSparkMax.follow(leftFrontSparkMax);
    // rightRearSparkMax.follow(rightFrontSparkMax);

    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);

    // m_leftLeader.setPID(DriveConstants.kp, 0, 0);
    // m_rightLeader.setPID(DriveConstants.kp, 0, 0);
    m_leftPidController = leftFrontSparkMax.getPIDController();
    m_rightPidController = rightFrontSparkMax.getPIDController();
    m_leftPidController.setP(DriveConstants.kp);
    m_leftPidController.setI(0);
    m_leftPidController.setD(0);
    m_rightPidController.setP(DriveConstants.kp);
    m_rightPidController.setI(0);
    m_rightPidController.setD(0);
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
   * Attempts to follow the given drive states using offboard PID.
   *
   * @param left The left wheel state.
   * @param right The right wheel state.
   */
  public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
    // m_leftLeader.setSetpoint(
    //     ExampleSmartMotorController.PIDMode.kPosition,
    //     left.position,
    //     m_feedforward.calculate(left.velocity));
    // m_rightLeader.setSetpoint(
    //     ExampleSmartMotorController.PIDMode.kPosition,
    //     right.position,
    //     m_feedforward.calculate(right.velocity));

    m_leftPidController.setReference(left.position + m_feedforward.calculate(left.velocity), ControlType.kPosition);
    m_rightPidController.setReference(-(right.position + m_feedforward.calculate(right.velocity)), ControlType.kPosition);  // !!! I should not have to invert the right side.  This will mess up a bunch of stuff with Ramsete.  Must be a bug somewhere.
  }

  /**
   * Returns the left encoder distance.
   *
   * @return the left encoder distance
   */
  // public double getLeftEncoderDistance() {
  //   return m_leftLeader.getEncoderDistance();
  // }
  // public RelativeEncoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  /**
   * Returns the right encoder distance.
   *
   * @return the right encoder distance
   */
  // public double getRightEncoderDistance() {
  //   return m_rightLeader.getEncoderDistance();
  // }
  // public RelativeEncoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /** Resets the drive encoders. */
  public void resetEncoders() {
    // m_leftLeader.resetEncoder();
    // m_rightLeader.resetEncoder();
    // Make sure we start with zero velocity
    m_leftPidController.setReference(0.0, ControlType.kPosition);
    m_rightPidController.setReference(0.0, ControlType.kPosition);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
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
