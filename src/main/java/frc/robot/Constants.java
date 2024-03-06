// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
// Miscellanous
    public static final double kStopSpeed = 0.0; // stop motor speed

// Auto
    public static final double kAutoWindupTime = 2;
    public static final double kAutoDivisor = 4;
    public static final double kPosAngle = 60;
    public static final double kNegAngle = -60;

// Climb Subsystem
   public static int kleftclimbID = 10;
   public static int krightclimbID = 9;

// Drivetrain Subsystem
    public static int kRightLeadID = 1; // CAN value of Right Lead Motor
    public static int kLeftLeadID = 3; // CAN value of Left Lead Motor
    public static int kRightFollowID = 2; // CAN value of Right Follow Motor
    public static int kLeftFollowID = 4; // CAN value of Left Follow Motor

    public static final int[] kLeftEncoderPorts = {0, 1};
    public static final int[] kRightEncoderPorts = {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kDriveSpeed = 3.0; // Update this with your desired drive speed
    public static final double kMaxAngularSpeed = Math.PI; // Update this with your desired turn speed
    public static final double kTrackWidth = 0.55245; // Update this with your desired track width
    public static final double kWheelRadius = 0.0762; // Update this with your desired wheel radius
    public static final double kEncoderResolution = 4096; // Update this with your desired resolution
    public static final double kWheelDiameterMeters = 0.152;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderResolution;
  
// Intake Subsystem
    public static int kHighIntakeID = 5; // CAN value of kHighIntakeID
    public static int kLowIntakeID = 6; // CAN value of kLowIntakeID
  
    public static final double kIntakeSpeed = .75; // Update this with your desired intake speed
    public static final double kIntakeIn = kIntakeSpeed;
    public static final double kIntakeOut = -kIntakeSpeed;

// Shooter Subsystem
    public static int kRightShooterID = 7; // CAN value of Right Shooter
    public static int kLeftShooterID = 8; // CAN value of Left Shooter

    public static final double kShooterSpeed = 0.95; // Update this with your desired shooter speed
    public static final double kShooterLaunch = kShooterSpeed;
    public static final double kShooterEject = -kShooterSpeed;
        
// Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
// Cameras
  public static final int cameraPort = 1;
  public static final int rampcameraPort = 0;
}