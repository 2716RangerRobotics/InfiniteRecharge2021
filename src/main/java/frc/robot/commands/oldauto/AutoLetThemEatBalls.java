/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.oldauto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoLetThemEatBalls extends SequentialCommandGroup {
  /**
   * Creates a new AutoLetThemEatBalls.
   */
  public AutoLetThemEatBalls() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveBrakeOn(),
      new DriveResetGyro(),
      new BallTiltOut(),
      new DriveStraightToDistance(.6096, .35, 0.0),
      new DriveResetEncoders(),
      new ParallelRaceGroup(
        new DriveStraightToDistance(2.54, .50, 0.0),
        new BallIntakeHandleOuttake()
      ),
      new DriveResetEncoders(),
      new DriveStraightToDistance(.508, .45, 0.0),
      new WaitCommand(.05),
      new DriveStraightToDistance(-3.6576, .45)
      );
  }
}
