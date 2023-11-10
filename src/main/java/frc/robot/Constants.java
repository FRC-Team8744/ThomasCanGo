// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftFrontCAN = 9;
    public static final int kLeftRearCAN = 10;
    public static final int kRightFrontCAN = 7;
    public static final int kRightRearCAN = 8;

    public static final double kConvertInchToMeter = 0.0254;

    public static final double kTrackwidthInches = 18.5;
    public static final double kTrackwidthMeters = kTrackwidthInches * kConvertInchToMeter;

    public static final double kWheelDiameterInches = 6.0;
    public static final double kGearRatio = 8.45;
    public static final double kWheelDiameterMeters = kWheelDiameterInches * kConvertInchToMeter;
    public static final double kUnitsPerRotation = (kWheelDiameterMeters * Math.PI);  // !!! Use this value in SysID!
    public static final double kEncoderDistancePerRevolution = (kWheelDiameterMeters * Math.PI) / (kGearRatio);

    public static final double kEncoderVelocityFactor = kEncoderDistancePerRevolution/60.0; // Convert RPM to meters/second

    public static final double kMoveP = 1;
    public static final double kMoveI = 0;
    public static final double kMoveD = 0;

    // SparkMax Velocity PID
    public static final double kP = 0.00001;
    public static final double kI = 0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.00018;
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    // double maxRPM = 5700;

    public static final double kMaxDriveOutput = 0.4;  // range 0 to 1

    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    // !!!alh
    public static final int kButtonA = 1;
    public static final int kButtonB = 2;
    public static final int kButtonX = 3;
    public static final int kButtonY = 4;
    public static final int kButtonLeftBumper = 5;
    public static final int kButtonRightBumper = 6;
  }
}
