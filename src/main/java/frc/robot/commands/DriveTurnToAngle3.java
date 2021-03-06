// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveTurnToAngle3 extends CommandBase {
  private double targetAngle;
  private double adjustedAngle;
  private Timer timer;
  /** Creates a new DriveTurnToAngle3. */
  public DriveTurnToAngle3(double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    addRequirements(RobotContainer.drive);
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.zeroAngle();
    RobotContainer.drive.resetSpinPID();
    this.adjustedAngle =  this.targetAngle + RobotContainer.drive.getAngle();
    timer.stop();
    timer.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.driveWithSpinPID(0.0, adjustedAngle);
    if(RobotContainer.drive.atSpinPIDSetpoint())
      timer.start();
    else{
      timer.stop();
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.arcadeDrive(0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return //timer.get() >= .01;
    RobotContainer.drive.atSpinPIDSetpoint();
    // return Math.abs(RobotContainer.drive.getAngle() - adjustedAngle) <= 1;
    // return false;
    }
}
