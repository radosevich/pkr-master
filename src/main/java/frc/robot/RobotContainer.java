// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Commands.AutoDoNothing;
import frc.robot.Commands.AutoLaunchNote;
import frc.robot.Commands.TeleopLaunchNote;

import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
// The robot's subsystems
  private final Drivetrain m_driveSubsystem = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climb m_climb = new Climb();

// The robot's controllers
  private final XboxController m_driverController = 
    new XboxController(Constants.kDriverControllerPort);
  private final Joystick m_operatorController = 
    new Joystick(Constants.kOperatorControllerPort);

// The robot's autonomous commands
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser;

// Representation of game field
  public final Field2d m_field = new Field2d();

  public RobotContainer() {
    // setup controller button bindings to commands
    configureBindings();

    // Teleop default command
    // Control the drive with split-stick arcade controls
    m_driveSubsystem.setDefaultCommand(
      new InstantCommand(() -> m_driveSubsystem.manualDrive(
        -m_driverController.getLeftY(), 
        -m_driverController.getRightX()), 
        m_driveSubsystem));
    
    // Setup autonomous select commands
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Do nothing", new AutoDoNothing());
    m_chooser.addOption("LaunchNoteAuto", new AutoLaunchNote(m_shooter, m_intake));
    //m_chooser.addOption("Autonomous Distance", new AutonomousDistance(m_drivetrain, m_Tebo));
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {
  // Configure the button bindings

    new JoystickButton(m_driverController, Button.kRightBumper.value)
    // Intake note into the robot
      .onTrue(new InstantCommand(() -> m_intake.IntakeRun(Constants.kIntakeIn)))
      .onFalse(new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed)));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    // Reverse the intake direction to clear a jam
      .onTrue(new InstantCommand(() -> m_intake.IntakeRun(Constants.kIntakeOut)))
      .onFalse(new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed)));

    new JoystickButton(m_driverController, Button.kA.value)
    // Launch the note
      .onTrue(new InstantCommand(() -> m_shooter.ShooterRun(Constants.kShooterLaunch)))
      .onFalse(new InstantCommand(() -> m_shooter.ShooterRun(Constants.kStopSpeed)));

    new JoystickButton(m_driverController, Button.kB.value)
    // Reverse the shooter in case of a jam
      .onTrue(new InstantCommand(() -> m_shooter.ShooterRun(Constants.kShooterEject)))
      .onFalse(new InstantCommand(() -> m_shooter.ShooterRun(Constants.kStopSpeed)));

    new JoystickButton(m_operatorController, ButtonType.kTrigger.value)
    // Bind the operator joystick trigger button to the launch command
      .onTrue(new TeleopLaunchNote(m_shooter, m_intake));
  }

  public Command getAutonomousCommand() {
    // The selected chooser command will run in autonomous
    m_autonomousCommand = m_chooser.getSelected();
    return m_autonomousCommand;
  }
}
