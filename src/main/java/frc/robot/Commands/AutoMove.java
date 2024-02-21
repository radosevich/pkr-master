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
    {new Pose2d(15.592, 6.547, new Rotation2d(1.963)), new Pose2d(13.75, 7.009, new Rotation2d(1.963))},
    {new Pose2d(15.206, 5.55, new Rotation2d(3.14)), new Pose2d(13.867, 5.56, new Rotation2d(-3.14))},
    {new Pose2d(15.549, 4.515, new Rotation2d(2.128)), new Pose2d(13.909, 4.082, new Rotation2d(-3.14))},
    {new Pose2d(0.866, 4.511, new Rotation2d(-0.993)), new Pose2d(2.623, 4.107, new Rotation2d(.087))},
    {new Pose2d(1.36, 5.566, new Rotation2d(0.038)), new Pose2d(2.722, 5.573, new Rotation2d(-0.01))},
    {new Pose2d(0.955, 6.612, new Rotation2d(1.03)), new Pose2d(2.633, 7.014, new Rotation2d(-0.0353))}
  };

  public AutoMove(
  // Creates a new Move command.
    Drivetrain drivetrain, 
    int waypointIndex,
    boolean direction) {
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
      m_startPoseIndex = 0;
      m_finalPoseIndex = 1;
    } else {
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
    m_drivetrain.manualDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.manualDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_trajectory.getTotalTimeSeconds();
  }
}