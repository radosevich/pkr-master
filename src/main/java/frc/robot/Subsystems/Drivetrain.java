package frc.robot.Subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;

public class Drivetrain extends SubsystemBase {
  // Motors
  private final CANSparkMax m_rightLead = 
    new CANSparkMax(Constants.kRightLeadID, MotorType.kBrushless);
  private final CANSparkMax m_leftLead = 
    new CANSparkMax(Constants.kLeftLeadID, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = 
    new CANSparkMax(Constants.kRightFollowID, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = 
    new CANSparkMax(Constants.kLeftFollowID, MotorType.kBrushless);

  // Encoders
  private final Encoder m_leftEncoder =
    new Encoder(
      Constants.kLeftEncoderPorts[0],
      Constants.kLeftEncoderPorts[1],
      Constants.kLeftEncoderReversed);
  private final Encoder m_rightEncoder =
    new Encoder(
      Constants.kRightEncoderPorts[0],
      Constants.kRightEncoderPorts[1],
      Constants.kRightEncoderReversed);

  // Gyro
  private final AnalogGyro m_gyro;
  
  // Differential Drive
  private final DifferentialDrive m_robotDrive = 
    new DifferentialDrive(m_rightLead, m_leftLead);

  // PIDcontrollers
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;

  // Kinematics and odometry
  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward;

  private Translation2d m_translation;

  public Drivetrain() {
  //Creates a new drivetrain
    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);

    m_rightLead.setInverted(true);
  
    m_gyro = new AnalogGyro(0);

    m_leftPIDController = new PIDController(8.5, 0, 0); 
    m_rightPIDController = new PIDController(8.5, 0, 0);
  
    m_kinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), 
        m_leftEncoder.getDistance(), 
        m_rightEncoder.getDistance());

    m_feedforward = new SimpleMotorFeedforward(1, 3);
  }

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput = m_leftPIDController.calculate(
      m_leftEncoder.getRate(), 
      speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(
      m_rightEncoder.getRate(), 
      speeds.rightMetersPerSecond);
    m_leftLead.setVoltage(leftOutput + leftFeedforward);
    m_rightLead.setVoltage(rightOutput + rightFeedforward);
  } 

  public void drivePercent(double left, double right) {
    // Deadband gamepad 10%
    if (Math.abs(left) < 0.1) {left = 0.0;}
    if (Math.abs(right) < 0.1) {right = 0.0;}
    m_leftLead.set(left);
    m_rightLead.set(right);
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      m_gyro.getRotation2d(), 
      m_leftEncoder.getDistance(), 
      m_rightEncoder.getDistance(), 
      pose);
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    m_odometry.update(
      m_gyro.getRotation2d(), 
      m_leftEncoder.getDistance(), 
      m_rightEncoder.getDistance());
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void manualDrive(double move, double turn) {
    if (Math.abs(move) < 0.1) move = 0;
    if (Math.abs(turn) < 0.1) turn = 0;

    m_robotDrive.arcadeDrive(move, -turn);
  } 
}