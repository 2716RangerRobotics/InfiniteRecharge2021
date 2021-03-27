// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveToSensorDistance2 extends CommandBase {
  private TrapezoidProfile m_profile;
  private final Timer m_timer = new Timer();
  double kP = Constants.DRIVE_P_VALUE;
	double kD = Constants.DRIVE_D_VALUE;
  double kV = Constants.DRIVE_V_VALUE;
  double distance;
  double prevDistance;
  double prevTime;
  /** Creates a new DriveToSensorDistance2. */
  public DriveToSensorDistance2(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
     // The motion profile to be executed
    this.distance = distance;
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_profile = new TrapezoidProfile(
      // The motion profile constraints
      new TrapezoidProfile.Constraints(Constants.MAX_SPEED,Constants.MAX_ACCELERATION),
      // Goal state
      new TrapezoidProfile.State(distance,0),
      // Initial state
      new TrapezoidProfile.State(RobotContainer.shooter.getDistance(), 0));
    prevDistance = RobotContainer.shooter.getDistance();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentVelocity = (RobotContainer.shooter.getDistance() - prevDistance)/(m_timer.get() - prevTime);
    TrapezoidProfile.State reference = m_profile.calculate(m_timer.get());
    double accel = (m_profile.calculate(m_timer.get()+.01).velocity
                  - m_profile.calculate(m_timer.get()-.01).velocity)/.02;

    double feedForward = RobotContainer.drive.feedforward.calculate(reference.velocity,accel);
    double speedOutputLeft = RobotContainer.drive.leftPIDController.calculate(
            currentVelocity, reference.velocity);
    double speedOutputRight = RobotContainer.drive.rightPIDController.calculate(
            currentVelocity, reference.velocity);
    double posErrLeft = reference.position - RobotContainer.shooter.getDistance();
    double posErrRight = reference.position - RobotContainer.shooter.getDistance();
    double velErrLeft = reference.velocity - currentVelocity;
    double velErrRight = reference.velocity - currentVelocity;
    
    double outputLeft = ((kP * posErrLeft) + (kD * velErrLeft) + speedOutputLeft + feedForward)/12;
    double outputRight = ((kP * posErrRight) + (kD * velErrRight) + speedOutputRight + feedForward)/12;
      // + (kV *reference.velocity);

    // double output2 = RobotContainer.drive.rightPIDController.calculate(RobotContainer.drive.getLeftVelocity(), speeds.rightMetersPerSecond);
		RobotContainer.drive.setLeftRightMotorOutputs(outputLeft, outputRight);
    // RobotContainer.drive.driveWithSpinPID(output, targetAngle);
    System.out.println("time:" + m_timer.get() + " gPos: " + reference.position + " lPos: " + RobotContainer.drive.getLeftPosition() + 
            " rPos: " + RobotContainer.drive.getRightPosition());
    prevDistance = RobotContainer.shooter.getDistance();
    prevTime = m_timer.get();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){ 
    m_timer.stop();
    RobotContainer.drive.arcadeDrive(0, 0, false);
    // RobotContainer.drive.resetLeftEncoder();
    // RobotContainer.drive.resetRightEncoder();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_profile.totalTime());
  }

}
