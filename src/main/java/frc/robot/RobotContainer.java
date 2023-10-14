// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveDistanceProfiled;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Retained command references
  private final Command m_driveFullSpeed = Commands.runOnce(() -> m_robotDrive.setMaxOutput(1));
  private final Command m_driveHalfSpeed = Commands.runOnce(() -> m_robotDrive.setMaxOutput(0.5));

  // The driver's controller
//   CommandXboxController m_driverController =
//       new CommandXboxController(OIConstants.kDriverControllerPort);
    //   public final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    public final CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        Commands.run(
            () ->
                m_robotDrive.arcadeDrive(
                    // -m_driverController.getLeftY(), -m_driverController.getRightX()),
                    -m_driverController.getY(), -m_driverController.getX()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the bumper is held
    // m_driverController.rightBumper().onTrue(m_driveHalfSpeed).onFalse(m_driveFullSpeed);
    m_driverController.button(OIConstants.kButtonRightBumper).onTrue(m_driveHalfSpeed).onFalse(m_driveFullSpeed);
    // new JoystickButton(m_driverController, OIConstants.kButtonRightBumper)
    //     .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.1)))
    //     .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.2)));

    // Drive forward by 3 meters when the 'A' button is pressed, with a timeout of 10 seconds
    // m_driverController.a().onTrue(new DriveDistanceProfiled(3, m_robotDrive).withTimeout(10));
    m_driverController.button(OIConstants.kButtonA).onTrue(new DriveDistanceProfiled(3, m_robotDrive).withTimeout(10));

    // Do the same thing as above when the 'B' button is pressed, but defined inline
    m_driverController
        // .b()
        .button(OIConstants.kButtonB)
        .onTrue(
            new TrapezoidProfileCommand(
                    new TrapezoidProfile(
                        // Limit the max acceleration and velocity
                        new TrapezoidProfile.Constraints(
                            DriveConstants.kMaxSpeedMetersPerSecond,
                            DriveConstants.kMaxAccelerationMetersPerSecondSquared),
                        // End at desired position in meters; implicitly starts at 0
                        new TrapezoidProfile.State(3, 0)),
                    // Pipe the profile state to the drive
                    setpointState -> m_robotDrive.setDriveStates(setpointState, setpointState),
                    // Require the drive
                    m_robotDrive)
                .beforeStarting(m_robotDrive::resetEncoders)
                .withTimeout(10));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
