package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Commands.AutoIntakeRun;

public class CenterDrive extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param m_Drivetrain 
   * @param m_intake 
   * @param m_shooter The drivetrain subsystem on which this command will run
   */
  public CenterDrive(Drivetrain m_Drivetrain, Intake m_intake, Shooter m_shooter) {
    addCommands(
        new AutoLaunchNote(m_shooter, m_intake),
        new DriveDistance(-0.5, 65, m_Drivetrain),
        new AutoIntakeRun(m_intake),
        new DriveDistance(-0.5, -65, m_Drivetrain),
        new AutoLaunchNote(m_shooter, m_intake));

  }
}