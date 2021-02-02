// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A one wheel shooter class
 */
public class Shooter extends SubsystemBase {
  TalonSRX shooterWheel;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterWheel = new TalonSRX(Constants.SHOOTER_WHEEL);
    shooterWheel.configFactoryDefault();
    shooterWheel.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    shooterWheel.configVoltageCompSaturation(12.5);
    shooterWheel.enableVoltageCompensation(true);
    
    //this needs to be true if wheels is spinning the wrong way
    shooterWheel.setInverted(false);
    //if wheel spin is bawards from sensor, this must be true
    shooterWheel.setSensorPhase(false);
    shooterWheel.setNeutralMode(NeutralMode.Coast);
    shooterWheel.config_kP(0, 1.0);
    shooterWheel.config_kI(0, 0.0);
    shooterWheel.config_kD(0, 0.0);
    shooterWheel.config_kF(0, 0.0);

    // shooterWheel.configPeakOutputForward(1.0);
    // shooterWheel.configPeakOutputReverse(1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * returns the speed of the wheel
   * @return this is in native units, per 100ms
   */
  public double getSpeed(){
    return shooterWheel.getSelectedSensorVelocity();
  } 

  /**
   * set the speed of the wheel
   * @param speed this speed is in native units, per 100ms
   */
  public void setSpeed(double speed){
    shooterWheel.set(ControlMode.Velocity, speed);
  }

  /**
   * This method sets the shooter motor to zero volts in percent output
   */
  public void stop(){
    shooterWheel.set(ControlMode.PercentOutput, 0.0);
  }
}
  