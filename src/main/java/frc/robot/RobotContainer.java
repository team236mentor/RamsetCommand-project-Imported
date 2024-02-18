// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
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
  private Trajectory exampleTrajectory;

    private static DigitalInput autoSwitch1 = new DigitalInput(Constants.AutoConstants.DIO_AUTO_1);
    private static DigitalInput autoSwitch2 = new DigitalInput(Constants.AutoConstants.DIO_AUTO_2);
    private static DigitalInput autoSwitch3 = new DigitalInput(Constants.AutoConstants.DIO_AUTO_3);
    private static DigitalInput autoSwitch4 = new DigitalInput(Constants.AutoConstants.DIO_AUTO_4);

  // The driver's controller
  private XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(Constants.DriveConstants.kTrackwidthMeters);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

     /**
     * Configure default commands
     * call method to Set the default robot command as drive implimenting a arcade drive  */
    drive.setDefaultCommand( new RunCommand( () -> drive.arcadeDrive( -driverController.getLeftY(),-driverController.getRightX())));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // original code to Drive at half speed when the right bumper is held
    // creates new JoystickButton instead of  
    //      new JoystickButton(driverController, Button.kRightBumper.value)
    //          .onTrue(new InstantCommand(() ->drive.setMaxOutput(0.5)))
    //          .onFalse(new InstantCommand(() ->drive.setMaxOutput(1)));

    // CREATE BUTTONS
         // *XBOXCONTROLLER - DRIVER CONTROLLER
    JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
    JoystickButton y = new JoystickButton(driverController, Constants.XboxController.Y);
    JoystickButton lb = new JoystickButton(driverController, Constants.XboxController.LB);
    JoystickButton rb = new JoystickButton(driverController, Constants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driverController, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driverController, Constants.XboxController.RM);
    JoystickButton view = new JoystickButton(driverController, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driverController, Constants.XboxController.MENU);
    POVButton upPov = new POVButton(driverController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driverController,Constants.XboxController.POVXbox.DOWN_ANGLE); 
    POVButton leftPov = new POVButton(driverController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov = new POVButton(driverController,Constants.XboxController.POVXbox.RIGHT_ANGLE);

    // XBOX CONTROLLER - AUX CONTROLLER
    // JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
    // JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    // JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
    // JoystickButton y1 = new JoystickButton(auxController, Constants.XboxController.Y);
    // JoystickButton lb1 = new JoystickButton(auxController, Constants.XboxController.LB);
    // JoystickButton rb1 = new JoystickButton(auxController, Constants.XboxController.RB);
    // JoystickButton lm1 = new JoystickButton(auxController, Constants.XboxController.LM);
    // JoystickButton rm1 = new JoystickButton(auxController, Constants.XboxController.RM);
    // JoystickButton view1 = new JoystickButton(auxController, Constants.XboxController.VIEW);
    // JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);
    // POVButton upPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.UP_ANGLE);
    // POVButton downPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.DOWN_ANGLE);
    // POVButton leftPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    // POVButton rightPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.RIGHT_ANGLE);

       //assign button to comnands
    //***** driver controller ******
//X
    // x.onTrue(toggleGear);
    //   x.onTrue(stowTilt);
    //   x.onTrue(pidToTop);
//A
    // a.onTrue(podiumTilt);
    //   a.whileTrue(manualDown);
    //   a.onTrue(new AmpCameraAngle(ampTrapShooter));  
//B
    // b.whileTrue(setIntakeSpeed);
    //   b.onTrue(new FloorCameraAngle(ampTrapShooter));
    //   b.onTrue(pidToBot);
//Y
    // y.onTrue(wooferTilt);
    //  y.whileTrue(manualUp);

    // rb.onTrue(autoPIDDrive);
        rb.onTrue(new InstantCommand(() ->drive.setMaxOutput(0.5)));
        rb.onFalse(new InstantCommand(() ->drive.setMaxOutput(1.0)));
    // lb.onTrue(autoPIDTurn1);
    
//POV
    // upPov.whileTrue(llTarget);
    //   upPov.onTrue(floorCameraAngle);
        // downPov.whileTrue(llDistance);
    //  downPov.onTrue(ampCameraAngle);
    // leftPov.whileTrue(llAngle);

    //***** Aux Controller ******
//X
   //  x1.whileTrue(pidWooferSpeed);//cartridge motors only
   //   x1.onTrue(pidWooferShot); //intake and cart motors, also tilt
   //   x1.onTrue(frontShootGrabShoot);
//A
   //  a1.whileTrue(manualPodiumSpeed);
//B

   //  b1.whileTrue(pidPodiumSpeed);//cartridge motors only
//Y
   //  y1.whileTrue(manualWooferSpeed);
   //  y1.onTrue(pidPodiumShot); //intake and cart motors, also tilt

//POV
   //  upPov1.whileTrue(ampMotorForward);
   //    upPov1.whileTrue(manualIntake);
   //  downPov1.whileTrue(ampMotorReverse);
   //    downPov1.whileTrue(manualEject);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An choice of example trajectory base 
    // IMPORTANT : All units in meters
    SmartDashboard.putBoolean("current switch 1:",autoSwitch1.get());
    SmartDashboard.putBoolean("current switch 2:",autoSwitch2.get());
    SmartDashboard.putBoolean("current switch 3:",autoSwitch3.get());
    SmartDashboard.putBoolean("current switch 4:",autoSwitch4.get());

    if (autoSwitch1.get() == true ) {
            //option 1 for driving 
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                   new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(0))
                 , new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(-30))  )
               , new Pose2d(Units.inchesToMeters(90),Units.inchesToMeters(0), new Rotation2d(0)),
            // Pass config
            config);
    } else if (autoSwitch1.get() == true  ) {
   //option 2 for driving 
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                   new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(0))
                 , new Translation2d(Units.inchesToMeters(20), Units.inchesToMeters(10))
                 , new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(-10))  )
               , new Pose2d(Units.inchesToMeters(40),Units.inchesToMeters(20), new Rotation2d(0)),
            // Pass config
            config);
    } else  {
       exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(180)),
            // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                   new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(30))
                 , new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(-30))  )
               , new Pose2d(Units.inchesToMeters(90),Units.inchesToMeters(0), new Rotation2d(0)),
            // Pass config
            config);
    }


    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
        drive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
        drive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.runOnce(() ->drive.SetNewPose(exampleTrajectory.getInitialPose()))
        .andThen(ramseteCommand)
        .andThen(Commands.runOnce(() ->drive.tankDriveVolts(0, 0)));
  }
}
