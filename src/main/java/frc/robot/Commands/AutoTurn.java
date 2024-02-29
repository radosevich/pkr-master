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
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Commands.AutoIntakeRun;

public class AutoTurn extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param m_Drivetrain 
   * @param m_intake 
   * @param m_shooter
   * @param degrees The drivetrain subsystem on which this command will run
   */
  public AutoTurn(double degrees, Drivetrain m_Drivetrain, Intake m_intake, Shooter m_shooter) {
    addCommands(
        new AutoLaunchNote(m_shooter, m_intake),
        new DriveDistance(-0.5, 28, m_Drivetrain),
        new TurnDegrees(-0.5, degrees, m_Drivetrain),
        new AutoIntakeRun(m_intake),
        // new InstantCommand(() -> m_intake.IntakeRun(Constants.kIntakeIn)),
        new DriveDistance(-0.5, 68, m_Drivetrain),
        new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed)),
        new DriveDistance(-0.5, -68, m_Drivetrain),
        new TurnDegrees(-0.5, -degrees, m_Drivetrain),
        new DriveDistance(-0.5, -28, m_Drivetrain),
        new AutoLaunchNote(m_shooter, m_intake));
  }
}
