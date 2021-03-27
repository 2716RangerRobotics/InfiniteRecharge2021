// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_ShootBalls extends SequentialCommandGroup {
  /** Creates a new ShootBalls. */
  public CG_ShootBalls(double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BallTiltIn(),//double check that the tilt is in
      new ShooterSetSpeed(speed),//get the shooter up to speed
      new CoDriverIntakeRumble(),//this does nothing
      new ParallelRaceGroup(
        new BallHandleIntake(),//move ball one and two forward...
        new ShooterWaitTilBall()//...till a ball slows that shooter
      ),
      new BallIntakeHandleStop(),//now that ball one in shooter, stop the other balls
      new ParallelCommandGroup(
        new ShooterSetSpeed(speed),//get the shooter up to speed, but while doing so also...
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new BallHandleIntakeTilBall(),//move balls forward till ball two ready to fire
            new BallIntakeIntake()//don't forget move ball in intake section as well
          ),
          new BallIntakeHandleStop()//now that the ball is in position to shoot, stop the balls
        )
      ),//now shoter is up to speed, continue on to
      new WaitCommand(0.0),//wait for ball two to stop
      new ParallelRaceGroup(
        new BallHandleIntake(),//move balls forward...
        new ShooterWaitTilBall()//...until ball two in shooter
      ),
      new BallIntakeHandleStop(),// stop the last ball, now that two two in shooter
      new ParallelCommandGroup(
        new ShooterSetSpeed(speed),//get shooter up to speed
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new BallHandleIntakeTilBall(),//move last ball to shooting position...
            new BallIntakeIntake()//...run the intake to do that too
          ),
          new BallIntakeHandleStop()//stop the last ball when it is in ready to shoot position
        )
      ),
      new WaitCommand(.1),//wait for ball three to stop
      new ParallelRaceGroup(
        new BallHandleIntake(),//move ball forward...
        new ShooterWaitTilBall()//...until ball three in shooter
      ),
      new BallIntakeHandleStop(),//stop the ball handler, it is empty
      new ShooterSetSpeed(speed),//wait for ball to leave shooter, and shooter to get back to speed
      new ShooterStop()//stop the shooter
    );
  }
}
