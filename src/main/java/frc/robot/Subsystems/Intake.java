package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax m_highintake;
  CANSparkMax m_lowintake;

  public Intake() {
  /** Creates a new Intake. */
    m_highintake = new CANSparkMax(Constants.kHighIntakeID, MotorType.kBrushless);
    m_lowintake = new CANSparkMax(Constants.kLowIntakeID, MotorType.kBrushless);
  }
  
  public void IntakeRun(double speed) {
  //set motor speed  to intake, eject or stop
    m_highintake.set(-speed);
    m_lowintake.set(speed);
  }
}
   
