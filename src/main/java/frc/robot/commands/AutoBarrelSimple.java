// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBarrelSimple extends SequentialCommandGroup {
  /** Creates a new AutoBarrelSimple. */
  public AutoBarrelSimple() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraightToDistance(150, .5), //3.81 m 
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(150, .5), //3.81 m
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(120, .5), //3.048 m 
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(60, .5), //1.524 m
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(240, .5) //6.096 m
    );
  }
}
