// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class DriveStraightToDistanceTest extends CommandBase {
  double speed;
  /** Creates a new DriveStraightToDistanceTest. */
  public DriveStraightToDistanceTest(double speed) {
    addRequirements(RobotContainer.drive);
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.setLeftRightMotorOutputs(speed,speed);
    // leftMotorMaster.set(Constants.LEFT_MOTOR_MASTER_TEST);
    // rightMotorMaster.set(Constants.RIGHT_MOTOR_MASTER_TEST);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.setLeftRightMotorOutputs(0.0, 0.0);
    // leftMotorMaster.set(0.0);
    // rightMotorMaster.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
