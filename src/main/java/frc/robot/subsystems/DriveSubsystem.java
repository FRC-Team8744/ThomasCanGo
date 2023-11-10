// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  private RelativeEncoder m_leftEncoder = leftFrontSparkMax.getEncoder();
  private RelativeEncoder m_rightEncoder = rightFrontSparkMax.getEncoder();

  public SparkMaxPIDController m_leftPID = leftFrontSparkMax.getPIDController();
  public SparkMaxPIDController m_rightPID = rightFrontSparkMax.getPIDController();

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  final DifferentialDriveOdometry m_odometry;

  // Create Field2d for robot and trajectory visualizations.
  public Field2d m_field;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leftFrontSparkMax.restoreFactoryDefaults();
    rightFrontSparkMax.restoreFactoryDefaults();
    leftRearSparkMax.restoreFactoryDefaults();
    rightRearSparkMax.restoreFactoryDefaults();

    leftRearSparkMax.follow(leftFrontSparkMax);
    rightRearSparkMax.follow(rightFrontSparkMax);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityFactor);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityFactor);

    resetEncoders();

    // Set SparkMax PID coefficients
    // Left
    m_leftPID.setP(DriveConstants.kP);
    m_leftPID.setI(DriveConstants.kI);
    m_leftPID.setD(DriveConstants.kD);
    m_leftPID.setIZone(DriveConstants.kIz);
    m_leftPID.setFF(DriveConstants.kFF);
    m_leftPID.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
    // Right
    m_rightPID.setP(DriveConstants.kP);
    m_rightPID.setI(DriveConstants.kI);
    m_rightPID.setD(DriveConstants.kD);
    m_rightPID.setIZone(DriveConstants.kIz);
    m_rightPID.setFF(DriveConstants.kFF);
    m_rightPID.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);

    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
    
    // Update robot position on Field2d.
    m_field.setRobotPose(getPose());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder (M)", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder (M)", getRightEncoderPosition());

    updateOdometry();
        
    // Update robot position on Field2d.
    m_field.setRobotPose(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Velocity reference only takes RPM!
    m_leftPID.setReference(speeds.leftMetersPerSecond/DriveConstants.kEncoderVelocityFactor, CANSparkMax.ControlType.kVelocity);
    m_rightPID.setReference(-speeds.rightMetersPerSecond/DriveConstants.kEncoderVelocityFactor, CANSparkMax.ControlType.kVelocity);

    m_drive.feed();  // Required!  Feed what?

    SmartDashboard.putNumber("Motor Command", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Motor Out", leftFrontSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Motor Velocity", m_leftEncoder.getVelocity());
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param speed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void driveVelocity(double speed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /**
   * Attempts to follow the given drive states using offboard PID.
   *
   * @param left The left wheel state.
   * @param right The right wheel state.
   */
  public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
    // Velocity reference only takes RPM!
    m_leftPID.setReference(left.velocity/DriveConstants.kEncoderVelocityFactor, CANSparkMax.ControlType.kVelocity);
    m_rightPID.setReference(-right.velocity/DriveConstants.kEncoderVelocityFactor, CANSparkMax.ControlType.kVelocity);

    m_drive.feed();  // Required!  Feed what?
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
  }
  

  /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose);
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
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoderPosition() {
    return -m_rightEncoder.getPosition();
  }

  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity() {
    return -m_rightEncoder.getVelocity();
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
  // public void setMaxOutput(double maxOutput) {
  //   m_drive.setMaxOutput(maxOutput);  // Does not work if driving by velocity settings
  // }
}
