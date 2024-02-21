// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class TeleopLaunchNote extends Command {
  private final Shooter m_shooter;
  private final Intake m_intake;

  private boolean done;

  public TeleopLaunchNote(Shooter shooter, Intake intake) {
    m_shooter = shooter;
    m_intake = intake;

    addRequirements(m_shooter, m_intake);
  }

  
  @Override
  public void initialize() {
  // Called when the command is initially scheduled.
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Start up launch motors
    m_shooter.ShooterRun(Constants.kShooterLaunch);
    // start up intake motor
    m_intake.IntakeRun(Constants.kIntakeIn);
    //stop all motors
    m_intake.IntakeRun(Constants.kStopSpeed); 
    m_shooter.ShooterRun(Constants.kStopSpeed);
    done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (done) {
      return true;
    } else {
      return false;
    }
  }
}
