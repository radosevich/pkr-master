// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class AutoMove extends Command {
  private Timer m_timer = new Timer();
  private final boolean kForward = true;
  private final Drivetrain m_drivetrain;
  private final int m_waypointIndex;
  private final boolean m_direction;
  private int m_startPoseIndex;
  private int m_finalPoseIndex;
  private final RamseteController m_ramsete = new RamseteController();
  private Trajectory m_trajectory;

  // Example waypoint arrays
  private final Pose2d[][] waypointsArray = new Pose2d[][]{
  // Red Left  
    {new Pose2d(15.79, 6.71, new Rotation2d(2.094)), new Pose2d(13.91, 7.02, new Rotation2d(-3.14))},
  // Red Center
    {new Pose2d(15.13, 5.56, new Rotation2d(-3.14)), new Pose2d(13.867, 5.56, new Rotation2d(3.14))},
  // Red Right
    {new Pose2d(15.76, 4.42, new Rotation2d(-2.094)), new Pose2d(13.95, 4.11, new Rotation2d(3.14))},
  // Blue Left
    {new Pose2d(0.78, 4.39, new Rotation2d(-1.009)), new Pose2d(2.62, 4.11, new Rotation2d(0))},
  // Blue Center
    {new Pose2d(1.38, 5.56, new Rotation2d(0)), new Pose2d(2.75, 5.56, new Rotation2d(0))},
  // Blue Right
    {new Pose2d(0.81, 6.7, new Rotation2d(0.925)), new Pose2d(2.52, 7.0, new Rotation2d(0))}
  };

  public AutoMove(
    Drivetrain drivetrain, 
    int waypointIndex,
    boolean direction) {
    /** Creates a new Move command. */
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    m_waypointIndex = waypointIndex;
    m_direction = direction;

  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    TrajectoryConfig m_config = new TrajectoryConfig(2, 2);

    if (m_direction==kForward) {
      System.out.println("Move.initialize(kForward)");
      m_startPoseIndex = 0;
      m_finalPoseIndex = 1;
    } else {
      System.out.println("Move.initialize(kReverse)");
      m_startPoseIndex = 1;
      m_finalPoseIndex = 0;
      m_config.setReversed(true);
    }

    m_trajectory = TrajectoryGenerator.generateTrajectory(
      waypointsArray[m_waypointIndex][m_startPoseIndex],
      List.of(),
      waypointsArray[m_waypointIndex][m_finalPoseIndex],
      m_config
    );

    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void execute() {
    double elapsed = m_timer.get();
    State reference = m_trajectory.sample(elapsed);
    
    ChassisSpeeds speeds = m_ramsete.calculate(m_drivetrain.getPose(), reference);
    m_drivetrain.manualDrive(-speeds.omegaRadiansPerSecond/4.5, -speeds.vxMetersPerSecond/4.5);
    //m_drivetrain.manualDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    //System.out.println("Move.execute(" + m_drivetrain.getPose() + ")");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Move.end(" + interrupted + ")");
    m_drivetrain.manualDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_trajectory.getTotalTimeSeconds();
  }
}