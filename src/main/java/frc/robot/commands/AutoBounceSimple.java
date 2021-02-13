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
      new DriveStraightToDistance(1.524, .5), //1.524 m 60 in
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(1.524, .5), //1.524 m 60 in
      new DriveTurnToAngle(-30, .6),
      new DriveStraightToDistance(-3.404, -.5), //-3.404 m 134 in
      new DriveTurnToAngle(120, .6),
      new DriveStraightToDistance(.762, .5), //.762 m 30 in
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(3.048, .5), //3.048 m 120 in
      new DriveStraightToDistance(-3.048, -.5), //-3.048 m 120 in
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(2.286, .5), //2.286 m 90 in
      new DriveTurnToAngle(-90, .6),
      new DriveStraightToDistance(3.048, .5), //3.048 m 120 in
      new DriveStraightToDistance(-1.524, -.5), //-1.524 m 60 in
      new DriveTurnToAngle(90, .6),
      new DriveStraightToDistance(1.524, .5)); //1.524 m 60 in
  }
}
