// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandle;

public class BallHandleIntakeTilBall extends CommandBase {
  /** Creates a new BallHandleIntakeTilBall. */
  public BallHandleIntakeTilBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.ballHandle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.ballHandle.setUpperMotors(BallHandle.UpperState.kIn);
    RobotContainer.ballHandle.setUpperMotors(BallHandle.UpperState.kIn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.ballHandle.setUpperMotors(BallHandle.UpperState.kOff1);
    RobotContainer.ballHandle.setUpperMotors(BallHandle.UpperState.kOff2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.ballHandle.getBallSensor();
  }
}
