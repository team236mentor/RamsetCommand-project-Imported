// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunTrajectory;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drive drive = new Drive();
  private RunTrajectory runTrajectory = new RunTrajectory(drive);
  
  
  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

  // PATH FOLLOWING objects in Robot Container
  private RamseteCommand ramseteCommand;
  private TrajectoryConfig config;
  private Trajectory exampleTrajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    drive.setDefaultCommand( 
    // this is in-lue of a command class that defines the arcade inputs
    new RunCommand( 
       () -> drive.arcadeDrive( 
            -driverController.getLeftY()
            , driverController.getRightX() )
            , drive) );
            
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Drive at half speed when the right bumper is held
    new JoystickButton(driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand( () -> drive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand( () -> drive.setMaxOutput(1))); 

    // buttons not required using defaultCommand
      //JoystickButton buttonA = new JoystickButton(driverController, Button.kA.value);
      //buttonA.onTrue(runTrajectory);

      // JoystickButton buttonB = new JoystickButton(driverController, 2);
       
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return runTrajectory;

  }
}
