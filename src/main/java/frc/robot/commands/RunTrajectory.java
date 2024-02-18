// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class RunTrajectory extends Command {
  DriveSubsystem drive;
  Trajectory exampleTrajectory;
  TrajectoryConfig config;
  RamseteCommand ramseteCommand;

  // assume values will be in inches must convert to meters
  private double[] startPose, endPose = {0,0,0};
  private double[] trajectory1, trajectory2, trajectory3 = {0,0};
  // private double[] traject2 = {60,-30};
  // private double[] traject3 = {90,0};
  // private double[] EndPose = {120,0,0};

  private double[] metricStartPose, metricEndPose = {0,0,0};
  private double[] metricTrajectory1, metricTrajectory2, metricTrajectory3 = {0,0};

  /** Creates a new runTrajectory. 
 * @return */
  public RunTrajectory(DriveSubsystem drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO get the values of the trajectory path from SmartDashboard
      startPose = SmartDashboard.getNumberArray("StartPose",startPose);
    trajectory1 = SmartDashboard.getNumberArray("traject1",trajectory1);
    trajectory2 = SmartDashboard.getNumberArray("traject2",trajectory2);
    trajectory3 = SmartDashboard.getNumberArray("traject3",trajectory3);
        endPose = SmartDashboard.getNumberArray("StartPose",endPose);

    // convert the pose units from inches to metric 
    for (int i = 0; i < 2; i++) {
      metricStartPose[i] = Units.inchesToMeters(startPose[i]);
      metricEndPose[i] = Units.inchesToMeters(endPose[i]);
     }

     // convert trajectory units from inches to metric 
     for (int i = 0; i < 3; i++) {
      metricTrajectory1[i] = Units.inchesToMeters(trajectory1[i]);
      metricTrajectory2[i] = Units.inchesToMeters(trajectory2[i]);
      metricTrajectory3[i] = Units.inchesToMeters(trajectory3[i]);
     }

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
      exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(metricStartPose[0],metricStartPose[1], new Rotation2d(metricStartPose[2]) ) ,
        // Pass through these two interior waypoints, making an 's' curve path
        List.of( new Translation2d(metricTrajectory1[0], metricTrajectory1[1])  
            , new Translation2d(metricTrajectory2[0], metricTrajectory2[1])
            , new Translation2d(metricTrajectory3[0], metricTrajectory3[1]) )
        , new Pose2d( metricEndPose[0], metricEndPose[1], new Rotation2d(metricEndPose[2]))
        , config);

      ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            drive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive);
      }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
