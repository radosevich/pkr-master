// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
import frc.robot.Commands.AutoDoNothing;
import frc.robot.Commands.AutoLaunchNote;
import frc.robot.Commands.TeleopLaunchNote;
import frc.robot.Commands.AutoLaunchNote;

public class AutoParallel extends ParallelCommandGroup {
  private final Shooter m_shooter;
  private final Intake m_intake;

  /** Creates a new AutoParallel. */
  public AutoParallel() {
    m_shooter = new Shooter();
    m_intake = new Intake();
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     new AutoLaunchNote(m_shooter, m_intake));
  }
}
