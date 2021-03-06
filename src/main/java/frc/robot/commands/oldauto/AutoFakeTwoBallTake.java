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
public class AutoFakeTwoBallTake extends SequentialCommandGroup {
  /**
   * Creates a new AutoFakeTwoBallTake.
   */
  public AutoFakeTwoBallTake() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveBrakeOn(),
      new DriveResetGyro(),
      new DriveResetEncoders(),
      new DriveStraightToDistance(-1.7018, -.25),
      new DriveTurnToAngle(-90, .35),
      new DriveStraightToDistance(4.191, .3),
      new DriveResetGyro(),
      new DriveTurnToAngle(-90, .35),
      new DriveResetEncoders(),
      new ParallelRaceGroup(
        new DriveStraightToDistance(1.143, .35),
        new BallTiltToScore()
      ),
      new DriveResetEncoders(),
      new WaitCommand(.05),
      new ParallelCommandGroup(
        new DriveStraightToDistance(.1778, 25),
        new BallTiltToScore(),
        new BallIntakeHandleOuttake() //need help, doesnt run outtake
        )
    );
  }
}
//acts like we are going to take balls but instead shoot them
//starts in front of human player station roughly
