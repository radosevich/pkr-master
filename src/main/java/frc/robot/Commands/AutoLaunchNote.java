// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class AutoLaunchNote extends SequentialCommandGroup {
  private final Shooter m_shooter;
  private final Intake m_intake;

  /** Creates a new AutoLaunchNote. */
  public AutoLaunchNote(Shooter shooter, Intake intake) {
    m_shooter = shooter;
    m_intake = intake;

    // Add your commands in the addCommands() call
    addCommands(
      new ParallelCommandGroup(new AutoShooterRun(m_shooter, 2), new SequentialCommandGroup( new WaitCommand(2), new AutoIntakeRun(m_intake, 2))),
      //new InstantCommand(() -> m_intake.IntakeRun(Constants.kIntakeIn)),
      new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed)),
      new InstantCommand(() -> m_shooter.ShooterRun(Constants.kStopSpeed)));
  }
}
