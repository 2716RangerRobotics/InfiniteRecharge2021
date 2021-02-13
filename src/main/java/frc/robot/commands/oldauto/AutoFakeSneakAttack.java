/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.oldauto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoFakeSneakAttack extends SequentialCommandGroup {
  /**
   * Creates a new AutoFakeSneakAttack.
   */
  public AutoFakeSneakAttack() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveBrakeOn(),
      new DriveResetGyro(),
      new DriveTurnToAngle(90, .35),
      new DriveStraightToDistance(2.159, .3),
      new DriveResetGyro(),
      new DriveTurnToAngle(90, .35),
      new DriveResetEncoders(),
      new ParallelRaceGroup(
        new DriveStraightToDistance(2.921, .3),
        new BallTiltToScore()
        ),

        new WaitCommand(.05),
        new DriveResetEncoders(),
        new ParallelCommandGroup(
          new DriveStraightToDistance(.381, .3),
          new BallTiltToScore(),
          new BallIntakeHandleOuttake()
        )
    );
  }
  //set up infront of human player station so the balls are straight ahead of the robot
  //this fakes the sneak attack and goes to shoot at the lower port
}
