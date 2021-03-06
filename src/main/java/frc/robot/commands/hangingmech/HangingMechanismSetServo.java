/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hangingmech;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class HangingMechanismSetServo extends InstantCommand {
  /**
   * Creates a new HangingMechanismSetServo.
   */
  public HangingMechanismSetServo() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hangingMechanism); 
  }

  //Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.hangingMechanism.stopRightMotor();
  // } else if(RobotContainer.hangingMechanism.getRightEncoder()>Constants.HANGING_EXTENDING_POSITION-3000){
    RobotContainer.hangingMechanism.setRightServo();
    RobotContainer.hangingMechanism.setLeftServo();
  }
  // public void execute() {
    // RobotContainer.hangingMechanism.setLeftMotor(-0.10);
  // }
}
