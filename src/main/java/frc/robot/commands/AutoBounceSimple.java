// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBounceSimple extends SequentialCommandGroup {
  /** Creates a new AutoBounceSimple. */
  public AutoBounceSimple() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new DriveStraightToDistance(60, .5),
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(60, .5),
      new DriveTurnToAngle(-30, .6),
      new DriveStraightToDistance(-134, -.5),
      new DriveTurnToAngle(120, .6),
      new DriveStraightToDistance(30, .5),
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(120, .5),
      new DriveStraightToDistance(-120, -.5),
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(90, .5),
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(120, .5),
      new DriveStraightToDistance(-60, -.5),
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(60, .5));
  }
}