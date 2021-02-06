// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class DriveStraightToDistance2 extends CommandBase {
  private double distance;
	private double speed;
	private boolean isReverse;
  private boolean isAngleTargeted;
  private double targetAngle;
  /** Creates a new DriveStraightToDistance2. */

  	/**$
	 * Drive to distance
	 * @param distanceInInches Distance in "inches"
	 * @param speed Speed
	 */
  public DriveStraightToDistance2(double distanceInInches, double speed) {
    this.distance = distanceInInches;
    this.speed = speed;
    addRequirements(RobotContainer.drive);
    isAngleTargeted = false;
    if (speed < 0) isReverse = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public DriveStraightToDistance2(double distanceInInches, double speed, double angle) {
    this.distance = distanceInInches;
    this.speed = speed;
    addRequirements(RobotContainer.drive);
    this.targetAngle = angle;
    isAngleTargeted = true;
    if (speed < 0) isReverse = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.drive.zeroAngle();
    RobotContainer.drive.resetSpinPID();
    if(!isAngleTargeted){
      targetAngle = RobotContainer.drive.getAngle();
    }
    RobotContainer.drive.resetLeftEncoder();
    RobotContainer.drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(startAngle + ", " + drive.getAngle());
   RobotContainer.drive.driveWithSpinPID(speed, targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.arcadeDrive(0, 0, false);
    RobotContainer.drive.resetLeftEncoder();
    RobotContainer.drive.resetRightEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isReverse) {
      if (RobotContainer.drive.getLeftPosition() < distance || RobotContainer.drive.getRightPosition() < distance) {
        return true;
      }
    } else {
      if (RobotContainer.drive.getLeftPosition() > distance || RobotContainer.drive.getRightPosition() > distance) {
        return true;
      }
    }
    return false;
  }
}
