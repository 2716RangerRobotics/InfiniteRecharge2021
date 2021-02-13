// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import static edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraightToDistance3 extends CommandBase  {
  private double targetAngle = 0.0;
  private final TrapezoidProfile m_profile;
  private final Timer m_timer = new Timer();
  double kP = Constants.DRIVE_P_VALUE;
	double kD = Constants.DRIVE_D_VALUE;
	double kV = Constants.DRIVE_V_VALUE;

  public DriveStraightToDistance3(double distance) {
      // The motion profile to be executed
      m_profile = new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.MAX_SPEED,Constants.MAX_ACCELERATION),
            // Goal state
            new TrapezoidProfile.State(distance,0),
            // Initial state
            new TrapezoidProfile.State(0,0));
      addRequirements(RobotContainer.drive);
  }


  @Override
  public void initialize() {
    targetAngle = RobotContainer.drive.getAngle();
    m_timer.reset();
    m_timer.start();
    RobotContainer.drive.resetLeftEncoder();
    RobotContainer.drive.resetRightEncoder();
  }

  @Override
  public void execute() {
    double averPos = (RobotContainer.drive.getLeftPosition()+RobotContainer.drive.getRightPosition())/2;
    double averVel = (RobotContainer.drive.getLeftVelocity()+RobotContainer.drive.getRightVelocity())/2;
    TrapezoidProfile.State reference = m_profile.calculate(m_timer.get());

    System.out.println("time:" + m_timer.get() + " gPos: " + reference.position + " rPos: " + averPos
      + " gVel: " + reference.velocity + " rVel: " + averVel); 

    double feedForward = RobotContainer.drive.feedforward.calculate(reference.velocity);
    double speedOutput = RobotContainer.drive.leftPIDController.calculate(
            RobotContainer.drive.getLeftVelocity(), reference.velocity);
    double posErr = reference.position - averPos;
    double velErr = reference.velocity - averVel;
    
    double output = (kP * posErr) + (kD * velErr) + speedOutput + feedForward;
      // + (kV *reference.velocity);

    // double output2 = RobotContainer.drive.rightPIDController.calculate(RobotContainer.drive.getLeftVelocity(), speeds.rightMetersPerSecond);
			
    RobotContainer.drive.driveWithSpinPID(output, targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    RobotContainer.drive.arcadeDrive(0, 0, false);
    RobotContainer.drive.resetLeftEncoder();
    RobotContainer.drive.resetRightEncoder();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_profile.totalTime());
  }

}
