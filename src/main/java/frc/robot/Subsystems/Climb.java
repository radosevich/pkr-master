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
  /** Creates a new Intake. */
    m_leftclimb = new CANSparkMax(Constants.kleftclimbID, MotorType.kBrushless);
    m_rightclimb = new CANSparkMax(Constants.krightclimbID, MotorType.kBrushless);
  }
  
  public void ClimbRun(double getLeftY, double getRightY) {
  //set motor speed  to intake, eject or stop
    m_leftclimb.set(getLeftY); 
    m_rightclimb.set(getRightY);
  }
}
   
