// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToSensorDistance extends InstantCommand {
  double targetDistance;
  public DriveToSensorDistance(double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(RobotContainer.drive, RobotContainer.shooter);
    this.targetDistance = targetDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new DriveStraightToDistance3(RobotContainer.shooter.getDistance() - targetDistance).schedule();
  }
}
