// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBalls extends SequentialCommandGroup {
  /** Creates a new ShootBalls. */
  public ShootBalls(double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterSetSpeed(speed),
      new CoDriverIntakeRumble(),
      new ParallelRaceGroup(
        new BallHandleIntake(),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new BallIntakeIntake()
        ),
        new WaitCommand(5)
      ),
      new ShooterStop()

    );
  }
}
