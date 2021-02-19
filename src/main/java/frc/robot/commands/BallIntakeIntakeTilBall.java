// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallIntake;

public class BallIntakeIntakeTilBall extends CommandBase {
  /** Creates a new BallIntakeIntakeTilBall. */
  public BallIntakeIntakeTilBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.ballIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.ballIntake.setLowerMotors(BallIntake.LowerState.kIn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.ballIntake.setLowerMotors(BallIntake.LowerState.kOff1);
    RobotContainer.ballIntake.setLowerMotors(BallIntake.LowerState.kOff2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.ballIntake.getBallSensor();
  }
}
