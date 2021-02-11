// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DrivePathWeaver extends CommandBase {
  /** Creates a new DrivePathWeaver. */
  Trajectory trajectory = new Trajectory();
  Timer timer = new Timer();
  RamseteController ramsete = new RamseteController();
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(13.597);

  public DrivePathWeaver(String fileName) {
    // Use addRequirements() here to declare subsystem dependencies.
    String trajectoryJSON = "paths/"+ fileName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    RobotContainer.drive.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State reference = trajectory.sample(timer.get());
    ChassisSpeeds speeds = ramsete.calculate(RobotContainer.drive.getCurrentPos(), reference);
    // kinematics.toWheelSpeeds(speeds);
    // RobotContainer.drive.arcadeDrive(moveValue, rotateValue, squaredInputs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    /** Sets speeds to the drivetrain motors. */
    // public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    //   var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    //   var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    //   double leftOutput =
    //       m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    //   double rightOutput =
    //       m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
  
    //   m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    //   m_rightGroup.setVoltage(rightOutput + rightFeedforward);
    // }
}
