// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final int ID_LEFT_FRONT = 35;   //   35  left front   // 40 
    public static final int ID_LEFT_REAR = 34;    //  34  left rear     // 41
    public static final int ID_RIGHT_FRONT = 1;   //   1   right front  // 42
    public static final int ID_RIGHT_REAR = 32;   //  32  right rear    // 43

    public static final int[] kLeftEncoderPorts = new int[] {11, 10};   //DIO_LDRIVE_ENC_A,B = 18 , 19 // 11, 10
    public static final int[] kRightEncoderPorts = new int[] {6, 7};  //DIO_RDRIVE_ENC_A,B = 13 , 12   //  6,  7
    public static final boolean leftEncoderReversed = false;         
    public static final boolean rightEncoderReversed = true;         

    public static final double trackwidthMeters =  Units.inchesToMeters(24);    //  0.69;   
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackwidthMeters);

    public static final int encoderCPR = 128;    // bourns Inc ENS1J-B28-L00128L   //  2023 robot code lists 512 counts/ticks per revolution 
    public static final double wheelDiameterMeters = Units.inchesToMeters(6);  //  0.15;  
    public static final double encoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (wheelDiameterMeters * Math.PI) / (double) encoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;        // example 1.98 kVs/Meter
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;  // example 0.2 kV*Sec^2/Meter

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;                    // example 8.5 velocity Meters/sec
  }

  public static final class OIConstants {
    public static final int USB_DRIVECONTROLLER = 0;
    // public static final int USB_AUXCONTROLLER = 1;
  }

  public static final class AutoConstants {
    public static final double maxSpeedMetersPerSecond = 3;                // example 3 meters/sec
    public static final double maxAccelerationMetersPerSecondSquared = 1;  // example 1 Meter/Sec^2
  }

}
