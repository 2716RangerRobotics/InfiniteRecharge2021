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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DrivePathWeaver extends CommandBase {
  /** Creates a new DrivePathWeaver. */
  Trajectory trajectory = new Trajectory();
  Timer timer = new Timer();
  RamseteController ramsete = new RamseteController();
  boolean isReverse = false;
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(.0677); //between .1354 and .01354 like .0677 .07447

  public DrivePathWeaver(String fileName, boolean isReverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    String trajectoryJSON = "output/"+ fileName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    this.isReverse= isReverse; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.setReverse(this.isReverse);
    RobotContainer.drive.setBrakeMode(true);
    timer.reset();
    timer.start();
    RobotContainer.drive.resetOdometry(trajectory.getInitialPose());
  
    // if(isReverse){
    //   trajectory.transformBy(new Transform2d(new Translation2d(), new Rotation2d(Math.PI)) );
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State reference = trajectory.sample(timer.get());
    ChassisSpeeds speeds = ramsete.calculate(RobotContainer.drive.getCurrentPos(), reference);

    //  speeds.vyMetersPerSecond = 0.0;
    setSpeeds(kinematics.toWheelSpeeds(speeds));
    // RobotContainer.drive.arcadeDrive(moveValue, rotateValue, squaredInputs);
    System.out.print("Current Pos" + RobotContainer.drive.getCurrentPos().getX());
    // System.out.print(RobotContainer.drive.get)
    System.out.println("         Reference" + reference.poseMeters.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.setReverse(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() <= timer.get();
  }

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = RobotContainer.drive.feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = RobotContainer.drive.feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput =
        RobotContainer.drive.leftPIDController.calculate(
          RobotContainer.drive.getLeftVelocity(), speeds.leftMetersPerSecond);
    double rightOutput =
        RobotContainer.drive.rightPIDController.calculate(
          RobotContainer.drive.getRightVelocity(), speeds.rightMetersPerSecond);
  
    RobotContainer.drive.setLeftRightMotorOutputs((leftOutput + leftFeedforward),
      (rightOutput + rightFeedforward));
      
  }
}
