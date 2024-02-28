// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedLeft extends SequentialCommandGroup {
  private final Drivetrain m_Drivetrain;
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public RedLeft(Drivetrain m_Drivetrain) {
    addCommands(
        new DriveDistance(-0.5, 28, m_Drivetrain),
        new TurnDegrees(-0.5, 180, m_Drivetrain),
        new DriveDistance(-0.5, 10, Drivetrain),
        new TurnDegrees(0.5, 180, Drivetrain));
  }
}
