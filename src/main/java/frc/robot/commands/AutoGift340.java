// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGift340 extends SequentialCommandGroup {
  /** Creates a new AutoGift340. */
  public AutoGift340() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveBrakeOn(),
      new DriveResetGyro(),
      new DriveStraightToDistance(-.9, -.15, 0.0),
      new ParallelRaceGroup(
        new BallTiltToPass(), 
        new WaitCommand(6)
      ),
      new ParallelRaceGroup(
        new BallTiltToPass(), 
        new BallIntakeHandleOuttake().withTimeout(5)
      ),
        new BallIntakeHandleStop()

    );
  }
}
