// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX m_leftMotor;
  private final WPI_TalonSRX m_rightMotor;

  public Shooter() {
  // Creates a new Shooter
    m_leftMotor = new WPI_TalonSRX(Constants.kLeftShooterID);
    m_rightMotor = new WPI_TalonSRX(Constants.kRightShooterID);
  }

  public void ShooterRun(double speed) {
  // Set motor speed to launch, eject or stop
    m_leftMotor.set(speed);
    m_rightMotor.set(-speed); // Motors are facing opposite directions
  }
}
