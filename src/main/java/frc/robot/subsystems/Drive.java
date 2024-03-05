// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

public class Drive extends SubsystemBase {
  
  // motors on the left side of the drive.
  private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port,MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port,MotorType.kBrushless);
  
  // motors on the right side of the drive.
  private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.kRightMotor1Port,MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port,MotorType.kBrushless);

  // encoder channel A and B reverse, reversed direction
  private final Encoder leftEncoder =  new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // encoder channel A and B reverse, reversed direction
  private final Encoder rightEncoder =  new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);
  // The gyro sensor
  private final AHRS gyro = new AHRS();

  // PATH FOLLOWING: add DifferentialDrive to DRIVE
      // robot drive left and right motor setter
      private final DifferentialDrive drive = 
        new DifferentialDrive(leftLeader::set, rightLeader::set);

  // PATH FOLLOWING: add DifferentialDriveOdometry to DRIVE
      // Odometry class for tracking robot pose
      private final DifferentialDriveOdometry odometry;


  /** Creates a new DriveSubsystem. */
  public Drive() {
    SendableRegistry.addChild(drive, leftLeader);
    SendableRegistry.addChild(drive, rightLeader);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Invert one side of the drivetrain so that positive voltages
    rightLeader.setInverted(true);

    // Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();

  // PATH FOLLOWING: add odometry to DRIVE 
    odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  @Override
  public void periodic() {
    
  // PATH FOLLOWING: Update the odometry in the periodic block
    odometry.update(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

            LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
            double[] botposeRed = llresults.targetingResults.botpose;
            SmartDashboard.putNumberArray("Limelight Pose", botposeRed);
            //  double pipelineLatency = llresults.results.latency_pipeline;
            //  LimelightHelpers.LimelightTarget_Fiducial = llresults.results.targets_Fiducials;
  }
// PATH FOLLOWING: 
  /** @return The currently-estimated pose of the robot */
  public Pose2d getPose() { return odometry.getPoseMeters();  }

  // PATH FOLLOWING: 
  /** @return The current wheel speeds */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  // PATH FOLLOWING: 
  /** @param pose reset the pose to which to set the odometry */
  public void resetOdometry(Pose2d pose) {
      resetEncoders();
      odometry.resetPosition( gyro.getRotation2d()
          , leftEncoder.getDistance()
          , rightEncoder.getDistance()
          , pose );
    }

  /** Drives the robot using arcade controls.
  * @param fwd @param rot commanded forward and rotation movement */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  // PATH FOLLOWING:
  /** Controls the left and right sides of the drive directly with voltages.
  * @param leftVolts @param rightVolts  the commanded left and right output
  */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
      leftLeader.setVoltage(leftVolts);
      rightLeader.setVoltage(rightVolts);
      drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {  
      leftEncoder.reset();
      rightEncoder.reset();
  }

  /** Gets the average distance of the two encoders.
   * @return the average of the two encoder readings */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /** @return the left drive encoder */
  public Encoder getLeftEncoder() { return leftEncoder;  }

  /** @return the right drive encoder */
  public Encoder getRightEncoder() { return rightEncoder;  }

  /** Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   * @param maxOutput the maximum output to which the drive will be constrained  */
  public void setMaxOutput(double maxOutput) {  drive.setMaxOutput(maxOutput);  }

  /** Zeroes the heading of the robot */
  // public void zeroHeading() {
  //   gyro.reset();
  // }

  /** @return the robot's heading in degrees, from -180 to 180 */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /** @return The turn rate of the robot, in degrees per second */
    public double getTurnRate() {
    return -gyro.getRate();
  }

}
