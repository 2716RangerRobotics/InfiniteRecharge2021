/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.oldauto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoFeedShooter extends SequentialCommandGroup {
  /**
   * Creates a new AutoFeedShooter.
   */
  public AutoFeedShooter() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveBrakeOn(),
      new BallTiltOut(),
      new WaitCommand(5),
      new BallIntakeHandleOuttake().withTimeout(5),
      new BallTiltIn(),
      new DriveTurnToAngle(45, .25),
      new DriveStraightToDistance(25, -.25)
    );
  }
}
//works anywhere (it would be based off where team wants us)&turning 45 brings us to get balls
//passes balls to alliance members
