// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
// import frc.robot.commands.DriveDistProfiled;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  public final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    // m_robotDrive.setDefaultCommand( Commands.run( () -> m_robotDrive.arcadeDrive(-m_driverController.getY(), -m_driverController.getX()), m_robotDrive));
    m_robotDrive.setDefaultCommand( Commands.run( () -> m_robotDrive.driveVelocity(-m_driverController.getY(), -m_driverController.getX()), m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, OIConstants.kButtonRightBumper)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.2)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.kMaxDriveOutput)));

    // Drive forward by 1.5 meters when the 'A' button is pressed, with a timeout of 5 seconds
    new JoystickButton(m_driverController, OIConstants.kButtonA)
    .onTrue(new DriveDistance(1.5, m_robotDrive).withTimeout(5));
    
    // Drive backward by 1.5 meters when the 'B' button is pressed, with a timeout of 5 seconds
    new JoystickButton(m_driverController, OIConstants.kButtonB)
    .onTrue(new DriveDistance(-1.5, m_robotDrive).withTimeout(5));

    // Drive forward by 1.5 meters when the 'X' button is pressed, with a timeout of 5 seconds
    // new JoystickButton(m_driverController, OIConstants.kButtonX)
    // .onTrue(new DriveDistProfiled(1.5, m_robotDrive).withTimeout(5));
    
    // Drive backward by 1.5 meters when the 'Y' button is pressed, with a timeout of 5 seconds
    // new JoystickButton(m_driverController, OIConstants.kButtonY)
    // .onTrue(new DriveDistProfiled(-1.5, m_robotDrive).withTimeout(5));
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
