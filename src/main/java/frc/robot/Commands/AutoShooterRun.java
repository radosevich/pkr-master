// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;

public class AutoShooterRun extends Command {
  private Timer timer;
  private final Shooter m_shooter;
  private final double m_time;

  /** Creates a new AutoShooterRun. */
  public AutoShooterRun(Shooter shooter, double time) {
    System.out.println("AutoShooterRun()");
    timer = new Timer();
    m_shooter = shooter;
    m_time = time;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("AutoShooterRun " + timer.get());
    m_shooter.ShooterRun(Constants.kShooterLaunch);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return timer.get() >= m_time;
  }
}
