package frc.robot.Subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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
  
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    }
    
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
    }
    
  public int getRightEncoderCount() {
    return m_rightEncoder.get();
    }
    
    public double getLeftDistanceInch() {
      return m_leftEncoder.getDistance();
    }
  
    public double getRightDistanceInch() {
      return m_rightEncoder.getDistance();
    }
  
    public double getAverageDistanceInch() {
      return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    }
  // Gyro
  private final AnalogGyro m_gyro;
  
  // Differential Drive
  private final DifferentialDrive m_robotDrive = 
    new DifferentialDrive(m_leftLead, m_rightLead);

  public Drivetrain() {
  //Creates a new drivetrain
    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);

    m_rightLead.setInverted(true);
    m_rightFollow.setInverted(true);
    
    m_gyro = new AnalogGyro(0);
  }
  public void manualDrive(double move, double turn) {
    if (Math.abs(move) < 0.1) move = 0;
    if (Math.abs(turn) < 0.1) turn = 0;

    m_robotDrive.arcadeDrive(move, turn);
  } 
}