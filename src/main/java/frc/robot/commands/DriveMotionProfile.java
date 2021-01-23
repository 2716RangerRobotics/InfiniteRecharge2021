// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DriveMotionProfile extends CommandBase {
  private volatile boolean isFinished = false;
	private double[][] leftMotion;
	private double[][] rightMotion;
	private volatile int i = 0;
	private volatile double prevErrorL = 0;
	private volatile double prevErrorR = 0;
  /** Creates a new DriveMotionProfile. */
  public DriveMotionProfile(String filename) {
      addRequirement(RobotController.drive);
      this.leftMotion = MotionProfiles.loadProfile(filename + "_left");
      this.rightMotion = MotionProfiles.loadProfile(filename + "_right");
  }
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.resetLeftEncoder();
    RobotContainer.drive.resetRightEncoder();
		SmartDashboard.putBoolean("PathRunning", true);
		isFinished = false;
		i = 0;
		prevErrorL = 0;
		prevErrorR = 0;

		if (leftMotion.length != rightMotion.length) {
			System.out.println("Left and right profiles not of equal length!");
			this.cancel();
			return;
		}

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
		if (i == 24) {
			if (RobotContainer.drive.getLeftPosition() == 0) {
				DriverStation.reportError("yo man left encoder is dead man", false);
				// new DriveForTime(.5, 1.5).start();
			} else if (RobotContainer.drive.getRightPosition() == 0) {
				DriverStation.reportError("aw dang right encoder is chooched", false);
				// new DriveForTime(.5, 1.5).start();
			}
		}

		if (i < leftMotion.length) {
			double goalPosL = leftMotion[i][0];
			double goalVelL = leftMotion[i][1];
			double goalAccL = leftMotion[i][2];

			double goalPosR = rightMotion[i][0];
			double goalVelR = rightMotion[i][1];
			double goalAccR = rightMotion[i][2];

			double errorL = goalPosL - RobotContainer.drive.getLeftPosition();
			double errorDerivL = ((errorL - prevErrorL) / Constants.MOTION_PROFILE_PERIOD) - goalVelL;

			double errorR = goalPosR - RobotContainer.drive.getRightPosition();
			double errorDerivR = ((errorR - prevErrorR) / Constants.MOTION_PROFILE_PERIOD) - goalVelR;

			double kP = Constants.DRIVE_P_VALUE;
			double kD = Constants.DRIVE_D_VALUE;
			double kV = Constants.DRIVE_V_VALUE;
			double kA = Constants.DRIVE_A_VALUE;

			double pwmL = (kP * errorL) + (kD * errorDerivL) + (kV * goalVelL) + (kA * goalAccL);
			double pwmR = (kP * errorR) + (kD * errorDerivR) + (kV * goalVelR) + (kA * goalAccR);

			SmartDashboard.putNumber("TargetLeft", goalPosL);
			SmartDashboard.putNumber("ActualLeft", RobotContainer.drive.getLeftPosition());

			SmartDashboard.putNumber("TargetRight", goalPosR);
			SmartDashboard.putNumber("ActualRight", RobotContainer.drive.getRightPosition());

			prevErrorL = errorL;
			prevErrorR = errorR;

			RobotContainer.drive.setLeftRightMotorOutputs(pwmL, pwmR);
			i++;
		} else {
			isFinished = true;
		}
	}



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = true;
    SmartDashboard.putBoolean("PathRunning", false);
		RobotContainer.drive.setLeftRightMotorOutputs(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
