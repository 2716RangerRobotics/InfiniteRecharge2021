// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTurnToAngle2 extends PIDCommand {
  double initAngle = 0;
  /** Creates a new DriveTurnToAngle2. */
  public DriveTurnToAngle2(double angle) {
    super(
        // The controller that the command will use
        new PIDController(0.018, 0, 0.001),
        // This should return the measurement
        getErrorToTarget(),
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        useOutput() 
          // Use the output here
        );
    addRequirements(RobotContainer.drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  public double getErrorToTarget(){
    return RobotContainer.drive.getAngle() - initAngle;
  }
  public void useOutput(double output){
    RobotContainer.drive.arcadeDrive(moveSpeed, turnSpeed, false);
  }
  public void initialize() {
    initAngle = RobotContainer.drive.getAngle();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getErrorToTarget() - angle)  <= 1;
  }
}
