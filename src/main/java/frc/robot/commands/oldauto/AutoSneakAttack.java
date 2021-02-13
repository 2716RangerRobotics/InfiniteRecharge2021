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
public class AutoSneakAttack extends SequentialCommandGroup {
  /**
   * Creates a new AutoTrenchClaim.
   */
  public AutoSneakAttack() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
      super(
      new DriveBrakeOn(),
      new DriveResetGyro(),
      new ParallelCommandGroup(
        new BallTiltOut(),
        new DriveStraightToDistance(1.524, .45, 0.0)
      ),
      new DriveResetGyro(),      
      new ParallelRaceGroup(
        new BallIntakeIntake(),
        new BallHandleIntake(),
        new DriveStraightToDistance(2.5908, .35, 0.0)
      ),
      new DriveResetGyro(),
      new DriveResetEncoders(),
      new WaitCommand(.05),
      new ParallelCommandGroup(
        new DriveStraightToDistance(2.2352, .25, 0.0),
        new BallTiltIn()

      )
      //new BallIntakeIntake(),
     // new DriveTurnToAngle(180, .35),
     // new ParallelRaceGroup(
        //new DriveStraightToDistance(100, .35),
       // new BallTiltToScore()
     // )
      
    );
      // new 
      // new DriveStraightToDistance(distanceInInches, speed),
  }
}
