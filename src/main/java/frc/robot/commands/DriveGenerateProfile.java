// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveGenerateProfile extends CommandBase {
  private double[][] leftMotion;
  private double[][] rightMotion;
  private volatile boolean isFinished = false;
  private volatile int i = 0;
  private volatile double prevPosRight = 0.0;
  private volatile double prevPosLeft = 0.0;
  private volatile double prevVelocityRight = 0.0;
  private volatile double prevVelocityLeft = 0.0;

  /** Creates a new DriveGenerateProfile. */
  public DriveGenerateProfile(int totalLength,String filename) {
    addRequirements(RobotContainer.drive);
    leftMotion = new double[totalLength][3];
    rightMotion = new double[totalLength][3];  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.resetLeftEncoder();
    RobotContainer.drive.resetRightEncoder();
	  SmartDashboard.putBoolean("PathWriting", true);
	  isFinished = false;
	  i = 0;
	
	  new Thread(() -> {
		  double lastTime = 0;
		  while (!isFinished && DriverStation.getInstance().isEnabled()) {
			  if (Timer.getFPGATimestamp() >= lastTime + Constants.MOTION_PROFILE_PERIOD) {
				  lastTime = Timer.getFPGATimestamp();
				  threadedExecute();
		  	}
			  try {
				  Thread.sleep(2);
			  } catch (InterruptedException e) {
				  e.printStackTrace();
			  }
		  }
    }).start();
  }
  protected synchronized void threadedExecute() {
    if (i < leftMotion.length) {
      leftMotion[i][0] = RobotContainer.drive.getLeftPosition();
      leftMotion[i][1] = (leftMotion[i][0] - prevPosLeft) / Constants.MOTION_PROFILE_PERIOD;
      leftMotion[i][2] = (leftMotion[i][1] - prevVelocityLeft) / Constants.MOTION_PROFILE_PERIOD;
      rightMotion[i][0] = RobotContainer.drive.getRightPosition();
      rightMotion[i][1] = (rightMotion[i][1] - prevPosRight) / Constants.MOTION_PROFILE_PERIOD;
      rightMotion[i][2] = (rightMotion[i][1] - prevVelocityRight) / Constants.MOTION_PROFILE_PERIOD;
      prevPosLeft = leftMotion[i][0];
      prevPosRight = rightMotion[i][0];
      prevVelocityLeft = leftMotion[i][1];
      prevVelocityRight = rightMotion[i][1];
      i++;
    }else{
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    double driveValueFast = RobotContainer.getDriverLeftStickY();
    double turnValueFast = RobotContainer.getDriverLeftStickX();
    RobotContainer.drive.arcadeDrive(driveValueFast, turnValueFast, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
