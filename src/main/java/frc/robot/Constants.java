// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kLeftMotor1Port = 35;   //   35  left front 
    public static final int kLeftMotor2Port = 34;    //  34  left rear 
    public static final int kRightMotor1Port = 1;   //   1   right front 
    public static final int kRightMotor2Port = 32;   //  32  right rear

    
    public static final int[] kLeftEncoderPorts = new int[] {12, 13};   //  DIO_LDRIVE_ENC_A,B = 18 , 19  (roboRio ports are 0-9)
    public static final int[] kRightEncoderPorts = new int[] {10, 11};  //  DIO_RDRIVE_ENC_A,B = 13 , 12  (NAVX ports are 10-19)

    public static final boolean kLeftEncoderReversed = false;         
    public static final boolean kRightEncoderReversed = true;         

    public static final double kTrackwidthMeters =  Units.inchesToMeters(22);    //  0.69;   2023 robot 22inches

    public static final int kEncoderCPR = 128;    // measured as bourns Inc ENS1J-B28-L00128L   //  2023 robot code lists 512 counts/ticks per revolution   
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);  //  0.15;  
   
    // Assumes the encoders are directly mounted on the wheel shafts 
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
      //auto selector switches
      public static final int DIO_AUTO_1 = 0;
      public static final int DIO_AUTO_2 = 1;
      public static final int DIO_AUTO_3 = 2; //2
      public static final int DIO_AUTO_4 = 3;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class XboxController {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int VIEW = 7;
    public static final int MENU = 8;
    public static final int LM = 9;
    public static final int RM = 10;

    public static class AxesXbox {
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int LTrig = 2;
      public static final int RTrig = 3;
      public static final int RX = 4;
      public static final int RY = 5;
    }
    public class POVXbox {
      public static final int UP_ANGLE = 0;
      public static final int RIGHT_ANGLE = 90;
      public static final int DOWN_ANGLE = 180;
      public static final int LEFT_ANGLE = 270;
    }
  }
}
