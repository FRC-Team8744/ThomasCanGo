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

    public static final double kWheelDiameterInches = 6.0;
    public static final double kGearRatio = 8.45;
    public static final double kWheelDiameterMeters = kWheelDiameterInches * kConvertInchToMeter;
    // public static final double kEncoderDistancePerPulse =
    //     // Assumes the encoders are directly mounted on the wheel shafts
    //     (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    public static final double kUnitsPerRotation = (kWheelDiameterMeters * Math.PI);  // !!! Use this value in SysID!
    public static final double kEncoderDistancePerRevolution = (kWheelDiameterMeters * Math.PI) / kGearRatio;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // Data from SysId file: C:\Users\FabLab9\FRC2024\ThomasCanTrack\sysid_data\sysid_data20231029-134230.json
    public static final double ksVolts = 0.11161;
    public static final double kvVoltSecondsPerMeter = 2.2496;
    public static final double kaVoltSecondsSquaredPerMeter = 0.346;

    public static final double kp = 1;

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
