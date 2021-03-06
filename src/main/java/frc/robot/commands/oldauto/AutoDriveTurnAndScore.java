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
public class AutoDriveTurnAndScore extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveAndScore.
   */
  public AutoDriveTurnAndScore() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveBrakeOn(),
      new DriveResetGyro(),
      new DriveStraightToDistance(1.3208, .25, 0.0),
      new DriveTurnToAngle(90, .35),
      new DriveStraightToDistance(2.159, .25, 90),
      new DriveTurnToAngle(-90, .35),
      new DriveResetEncoders(),
      new WaitCommand(.05),
      new ParallelRaceGroup(
        new DriveStraightToDistance(1.524, .25, 0.0),
        new BallTiltToScore(),
        new WaitCommand(1.5)
      ),
      //new ParallelRaceGroup(
        //new BallTiltToScore(),
        //new BallIntakeHandleOuttake(),
        //new WaitCommand(5.0)
       // ),
      new ParallelCommandGroup(
        new DriveStraightToDistance(.2032, .25),
        new BallIntakeHandleOuttake(),
        new BallTiltToScore()
      ).withTimeout(5)
    );
  }
}
//this works if we are left of the port 13 ft.
//11 pt. plan
