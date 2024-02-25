// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCrescendo extends SequentialCommandGroup {
  boolean kForward = true;
  boolean kBackward = false;

  /** Creates a new AutoCrescendo. */
  public AutoCrescendo(
    Drivetrain p_drivetrain, 
    Intake p_intake, 
    Shooter p_shooter, 
    int p_waypointIndex) {

    addCommands(
    // Add your command sequence
      // launch note into speaker at subwoofer
      new AutoLaunchNote(p_shooter, p_intake),
      // move forward past line to pick up note
      new AutoMove(p_drivetrain, p_waypointIndex, kForward),
      // reverse back to subwoofer
      new AutoMove(p_drivetrain, p_waypointIndex, kBackward),
      // launch note into speaker at subwoofer
      new AutoLaunchNote(p_shooter, p_intake)
    );
  }
}
