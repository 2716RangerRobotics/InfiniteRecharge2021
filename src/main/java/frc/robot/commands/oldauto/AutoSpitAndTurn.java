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
public class AutoSpitAndTurn extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveStraight.
   */
  public AutoSpitAndTurn() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveBrakeOn(),
      new DriveResetGyro(),
      new BallTiltToPass().withTimeout(3.0),//adjust for partner's needs
      // new WaitCommand(3.0),
      new ParallelRaceGroup(
        new BallIntakeHandleOuttake(),
        new BallTiltToPass(),
        new WaitCommand(3.0)
      ),
      new BallIntakeHandleStop(),
      new ParallelCommandGroup(
        new DriveStraightToDistance(-12, -.35),
        new BallTiltIn()
      ),
      new DriveTurnToAngle(-90, .35),
      new DriveResetEncoders(),
      new WaitCommand(.05),
      new DriveStraightToDistance(50, .25)
    );
    // System.out.println("something");
  }
}
//works anywhere (test plan)
//5 pt. plan