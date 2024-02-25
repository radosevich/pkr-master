// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Climb extends SubsystemBase {
  CANSparkMax m_leftclimb;
  CANSparkMax m_rightclimb;

  public Climb() {
    m_leftclimb = new CANSparkMax(Constants.kleftclimbID, MotorType.kBrushless);
    m_rightclimb = new CANSparkMax(Constants.krightclimbID, MotorType.kBrushless);
  }
  
  public void ClimbRun(double leftspeed, double rightspeed) {
    if (Math.abs(leftspeed) < 0.1) leftspeed = 0;
    if (Math.abs(rightspeed) < 0.1) rightspeed = 0;

    m_leftclimb.set(leftspeed);   
    m_rightclimb.set(rightspeed);
  }
}
   
