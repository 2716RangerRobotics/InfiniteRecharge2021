// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_BallIntakeForShooting extends SequentialCommandGroup {
  /** Creates a new CG_BallIntakeForShooting. */
  public CG_BallIntakeForShooting() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ShooterSetSpeed(-2000),//sets shooter wheel backwards slowly to condition it
        new SequentialCommandGroup( //all of the intake functions are here 
          new ParallelRaceGroup(
            new BallIntakeIntake(),//run the intake in...
            new BallHandleIntakeTilBall()//...until the ball is ready to shoot
          ),
          new WaitCommand(.1),//waits for the second ball
          new BallIntakeIntakeTilBall()//draws 2nd ball until sensor
        )
      ),
      new ShooterStop()//now that the intake is done, stops shooter wheel from spinning
    );
  }
}
