// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCrescendo extends SequentialCommandGroup {
  private final Drivetrain m_drivetrain;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final int m_waypointIndex;

  boolean kForward = true;
  boolean kBackward = false;

  /** Creates a new AutoCrescendo. */
  public AutoCrescendo( 
    Drivetrain p_drivetrain,
    Intake p_intake, 
    Shooter p_shooter, 
    int p_waypointIndex) {
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_shooter = p_shooter;
    m_waypointIndex = p_waypointIndex;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoLaunchNote(m_shooter, m_intake),
      new InstantCommand(() -> m_shooter.ShooterRun(Constants.kStopSpeed)),

      new AutoMove(m_drivetrain, m_waypointIndex, kForward),
      new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed)),

      new AutoMove(m_drivetrain, m_waypointIndex, kBackward),
      new AutoLaunchNote(m_shooter, m_intake),
      
      new InstantCommand(() -> m_shooter.ShooterRun(Constants.kStopSpeed)),
      new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed))
    );
  }
}
