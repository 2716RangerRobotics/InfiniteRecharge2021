/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;

public class HangingMechanism extends SubsystemBase {
  TalonSRX leftHangingMotor;
  TalonSRX rightHangingMotor;
  Servo rightServo;
  Servo leftServo;
  //  Encoder leftHangingEncoder;
  //  Encoder rightHangingEncoder;

  static final double HANGING_EXTEND_POSITION = 1000000.0;
  static final double HANGING_RETRACT_POSITION = -1000000.0;

  public HangingMechanism() {
    leftServo = new Servo(Constants.HANGING_MECH_LEFT_SERVO);
    rightServo = new Servo(Constants.HANGING_MECH_RIGHT_SERVO);

    leftHangingMotor = new TalonSRX(Constants.CLIMBING_LEFT_MOTOR);
    leftHangingMotor.configFactoryDefault();
    leftHangingMotor.setNeutralMode(NeutralMode.Brake);
    leftHangingMotor.setInverted(true);
    rightHangingMotor = new TalonSRX(Constants.CLIMBING_RIGHT_MOTOR);
    rightHangingMotor.configFactoryDefault();
    rightHangingMotor.setNeutralMode(NeutralMode.Brake);
    rightHangingMotor.setInverted(true);
    leftHangingMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightHangingMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightHangingMotor.setSensorPhase(true);
    leftHangingMotor.configVoltageCompSaturation(12.5);
    leftHangingMotor.enableVoltageCompensation(true);
    rightHangingMotor.configVoltageCompSaturation(12.5);
    rightHangingMotor.enableVoltageCompensation(true);
    leftHangingMotor.setSelectedSensorPosition(
        Preferences.getInstance().getDouble("ClimbLeftEnc", 0.0));
    rightHangingMotor.setSelectedSensorPosition(
        Preferences.getInstance().getDouble("ClimbRightEnc", 0.0));
    
    // resetLeftEncoder();
    // resetRightEncoder();

    // leftHangingMotor.config_kP(0, 1.0);
    // leftHangingMotor.config_kI(0, 0.0);
    // leftHangingMotor.config_kD(0, 0.0);
    // leftHangingMotor.config_kF(0, 0.0);

    // leftHangingMotor.configPeakOutputForward(Constants.CLIMBING_EXTENDING_MOTOR_SPEED);
    // leftHangingMotor.configPeakOutputReverse(Constants.CLIMBING_EXTENDING_MOTOR_SPEED);

    // rightHangingMotor.config_kP(0, 1.0);
    // rightHangingMotor.config_kI(0, 0.0);
    // rightHangingMotor.config_kD(0, 0.0);
    // rightHangingMotor.config_kF(0, 0.0);

    // rightHangingMotor.configPeakOutputForward(Constants.CLIMBING_EXTENDING_MOTOR_SPEED);
    // rightHangingMotor.configPeakOutputReverse(Constants.CLIMBING_EXTENDING_MOTOR_SPEED);

    // leftHangingMotor.config_kP(0, 1.0);
    // leftHangingMotor.config_kI(0, 0.0);
    // leftHangingMotor.config_kD(0, 0.0);
    // leftHangingMotor.config_kF(0, 0.0);

    // leftHangingMotor.configPeakOutputForward(Constants.CLIMBING_RETRACTING_MOTOR_SPEED);
    // leftHangingMotor.configPeakOutputReverse(Constants.CLIMBING_RETRACTING_MOTOR_SPEED);

    // rightHangingMotor.config_kP(0, 1.0);
    // rightHangingMotor.config_kI(0, 0.0);
    // rightHangingMotor.config_kD(0, 0.0);
    // rightHangingMotor.config_kF(0, 0.0);

    // rightHangingMotor.configPeakOutputForward(Constants.CLIMBING_RETRACTING_MOTOR_SPEED);
    // rightHangingMotor.configPeakOutputReverse(Constants.CLIMBING_RETRACTING_MOTOR_SPEED);

    // tiltMotorLeft.configFactoryDefault();
    // tiltMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double leftEncValue = getLeftEncoder();
    if(leftHangingMotor.hasResetOccurred()){
      leftHangingMotor.setSelectedSensorPosition(
        Preferences.getInstance().getDouble("ClimbLeftEnc", leftEncValue));
    }else{
      SmartDashboard.putNumber("ClimbLeftEnc", leftEncValue);
      Preferences.getInstance().putDouble("ClimbLeftEnc", leftEncValue);
    }

    double rightEncValue = getRightEncoder();
    if(rightHangingMotor.hasResetOccurred()){
      rightHangingMotor.setSelectedSensorPosition(
        Preferences.getInstance().getDouble("ClimbRightEnc", rightEncValue));
    }else{
      SmartDashboard.putNumber("ClimbRightEnc", rightEncValue);
      Preferences.getInstance().putDouble("ClimbRightEnc", rightEncValue);
    }
    
    // SmartDashboard.putNumber("ClimbRightEnc", getRightEncoder());
    
  }

  public void hangingExtendDistance(){
    leftHangingMotor(ControlMode.PercentOutput, HANGING_EXTEND_POSITION);
    rightHangingMotor(ControlMode.PercentOutput, HANGING_EXTEND_POSITION);
  }

  private void rightHangingMotor(ControlMode percentoutput, double hangingExtendPosition) {
    if(getRightEncoder() >= HANGING_EXTEND_POSITION){
      stopRightMotor();
    }
  }

  private void leftHangingMotor(ControlMode percentoutput, double hangingExtendPosition) {
    if(getLeftEncoder() >= HANGING_EXTEND_POSITION){
      stopLeftMotor();
    }
  }

  public void hangingRetractDistance(){
    leftHangingMotorRetract(ControlMode.PercentOutput, HANGING_RETRACT_POSITION);
    rightHangingMotorRetract(ControlMode.PercentOutput, HANGING_RETRACT_POSITION);
  }

  private void rightHangingMotorRetract(ControlMode percentoutput, double hangingRetractPosition) {
    if(getRightEncoder() <= HANGING_RETRACT_POSITION){
      //stopRightMotor();
    }
  }

  private void leftHangingMotorRetract(ControlMode percentoutput, double hangingRetractPosition){
    if(getLeftEncoder() <= HANGING_RETRACT_POSITION){
      //stopLeftMotor();
    }
  }

  public void resetLeftEncoder() {
    leftHangingMotor.setSelectedSensorPosition(-350);
  }

  public void resetRightEncoder() {
    rightHangingMotor.setSelectedSensorPosition(-350);
  }
  public void setLeftEncoder() {
    leftHangingMotor.setSelectedSensorPosition(3000);
  }
  public void setRightEncoder() {
    rightHangingMotor.setSelectedSensorPosition(3000);
  }
  public double getLeftEncoder() {
    return leftHangingMotor.getSelectedSensorPosition();
  }
  public double getRightEncoder() {
    return rightHangingMotor.getSelectedSensorPosition();
  }
  public void setLeftMotor(double speed) {
    leftHangingMotor.set(ControlMode.PercentOutput, speed);
  }
  public void setLeftMotor(ControlMode mode, double value) {
    leftHangingMotor.set(mode, value);
  }

  public void setRightMotor(double speed){
    rightHangingMotor.set(ControlMode.PercentOutput, speed);
  }
  public void setRightMotor(ControlMode mode, double value) {
    rightHangingMotor.set(mode, value);
  }
  public void stopLeftMotor() {
    leftHangingMotor.set(ControlMode.PercentOutput, 0.0);
  }
  public void stopRightMotor() {
    rightHangingMotor.set(ControlMode.PercentOutput, 0.0);
  }
  public void setLeftServo() {
    leftServo.set(.5);
  }
  public void setRightServo() {
    rightServo.set(.5);
  }
  public void resetLeftServo() {
    leftServo.set(0.0);
  }
  public void resetRightServo() {
    rightServo.set(0.0);
  }

}
