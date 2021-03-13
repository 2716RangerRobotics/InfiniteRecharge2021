/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
// import jdk.javadoc.internal.doclets.toolkit.util.DocFinder.Output;

public class Drive extends SubsystemBase {
  CANSparkMax leftMotorMaster;
  CANSparkMax leftMotorFollower;
  CANSparkMax rightMotorMaster;
  CANSparkMax rightMotorFollower;
  CANEncoder rightEncoder;
  CANEncoder leftEncoder;
  Encoder leftDriveEnc;
  private AHRS imu;
  public static final double FeedForward = 0.03;//18;
  public static final double kProportion = 0.018;
  DigitalInput colorWheelLimit;
  private PIDController spinController;
  private DifferentialDriveOdometry odometry;
  private Pose2d currentPos; 
  public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.DRIVE_V_VALUE,Constants.kA);
  public PIDController leftPIDController = 
    new PIDController(Constants.DRIVE_FORWARD_P, Constants.DRIVE_FORWARD_I, Constants.DRIVE_FORWARD_D);
  public PIDController rightPIDController =
    new PIDController(Constants.DRIVE_FORWARD_P, Constants.DRIVE_FORWARD_I, Constants.DRIVE_FORWARD_D);
  public boolean isReverse= false;
  /**
   * Creates a new Drive.
   */
  public Drive() {
    imu = new AHRS(SPI.Port.kMXP);
    leftMotorMaster = new CANSparkMax(Constants.LEFT_MOTOR_MASTER, MotorType.kBrushless);
    leftMotorFollower = new CANSparkMax(Constants.LEFT_MOTOR_FOLLOWER, MotorType.kBrushless);
    rightMotorMaster = new CANSparkMax(Constants.RIGHT_MOTOR_MASTER, MotorType.kBrushless);
    rightMotorFollower = new CANSparkMax(Constants.RIGHT_MOTOR_FOLLOWER, MotorType.kBrushless);

    leftMotorMaster.restoreFactoryDefaults();
    leftMotorFollower.restoreFactoryDefaults();

    rightMotorMaster.restoreFactoryDefaults();
    rightMotorFollower.restoreFactoryDefaults();

    rightMotorMaster.setInverted(true);
    rightMotorFollower.setInverted(true);

    leftMotorFollower.follow(leftMotorMaster);
    rightMotorFollower.follow(rightMotorMaster);

    leftMotorMaster.enableVoltageCompensation(12.5);
    rightMotorMaster.enableVoltageCompensation(12.5);

    rightEncoder = rightMotorMaster.getEncoder();
    leftEncoder = leftMotorMaster.getEncoder();                                                                                             

    rightEncoder.setPositionConversionFactor(.04194);//.04509);//.04284); //1.6866 - meter conversion value
    leftEncoder.setPositionConversionFactor(.04194);//.04509);//.04284); //1.6866

    rightEncoder.setVelocityConversionFactor(.04194 / 60);
    leftEncoder.setVelocityConversionFactor(.04194 / 60);

    leftMotorMaster.setSmartCurrentLimit(Constants.STALL_LIMIT_DRIVE, Constants.FREE_LIMIT_DRIVE);
    leftMotorFollower.setSmartCurrentLimit(Constants.STALL_LIMIT_DRIVE, Constants.FREE_LIMIT_DRIVE);

    rightMotorMaster.setSmartCurrentLimit(Constants.STALL_LIMIT_DRIVE, Constants.FREE_LIMIT_DRIVE);
    rightMotorFollower.setSmartCurrentLimit(Constants.STALL_LIMIT_DRIVE, Constants.FREE_LIMIT_DRIVE);

    leftMotorMaster.setOpenLoopRampRate(Constants.RAMP_RATE);
    leftMotorFollower.setOpenLoopRampRate(Constants.RAMP_RATE);

    rightMotorMaster.setOpenLoopRampRate(Constants.RAMP_RATE);
    rightMotorFollower.setOpenLoopRampRate(Constants.RAMP_RATE);

    spinController = new PIDController(Constants.DRIVE_SPIN_P, Constants.DRIVE_SPIN_I, Constants.DRIVE_SPIN_D);
    spinController.setTolerance(0.01);
    // leftMotorMaster.setOpenLoopRampRate(rate);
    // colorWheelLimit = new DigitalInput(Constants.EXTEND_LIMIT);
    odometry = new DifferentialDriveOdometry(imu.getRotation2d());
    leftDriveEnc = new Encoder(Constants.LEFT_DRIVE_ENCODER_A, Constants.LEFT_DRIVE_ENCODER_B);
    leftDriveEnc.setDistancePerPulse(.000218);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(DriverStation.getInstance().isAutonomous()||DriverStation.getInstance().isTest()){
      SmartDashboard.putNumber("Gyro", (int)imu.getYaw());
      SmartDashboard.putNumber("DriveLeftEnc", getLeftPosition());
      SmartDashboard.putNumber("DriveRightEnc", getRightPosition());
      SmartDashboard.putNumber("new Encoder", leftDriveEnc.get());
    

    // }
    if(isReverse){
      currentPos = odometry.update(imu.getRotation2d(),//.rotateBy(new Rotation2d(Math.PI)),
        this.getRightPosition()*-1 ,this.getLeftPosition()*-1 );
    }else{
      currentPos = odometry.update(imu.getRotation2d(), this.getLeftPosition(), this.getRightPosition());
    }
    SmartDashboard.putNumber("XPos", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("YPos", odometry.getPoseMeters().getY());
  }

  public void setReverse(boolean isReverse){
    this.isReverse = isReverse; 
  }
  
  public void setBrakeMode(boolean brakeMode){
  
    if(brakeMode){
      leftMotorMaster.setIdleMode(IdleMode.kBrake);
      leftMotorFollower.setIdleMode(IdleMode.kBrake);
      rightMotorMaster.setIdleMode(IdleMode.kBrake);
      rightMotorFollower.setIdleMode(IdleMode.kBrake);
    }else{
      leftMotorMaster.setIdleMode(IdleMode.kCoast);
      leftMotorFollower.setIdleMode(IdleMode.kCoast);
      rightMotorMaster.setIdleMode(IdleMode.kCoast);
      rightMotorFollower.setIdleMode(IdleMode.kCoast);
    }
  }
  public Pose2d getCurrentPos(){
    return currentPos;
  }
  public void resetOdometry(Pose2d pos0){
    this.resetLeftEncoder();
    this.resetRightEncoder();
    odometry.resetPosition(pos0, imu.getRotation2d());
  }

  /**
   * currently not on robot
   * @return the state of the limit switch that touches the color wheel
   */
  public boolean isColorWheelLimit() {
  //   if(!colorWheelLimit.get()){
  //     return true;
  //   }
  //   else{
      return false;
  }

  public void driveWithSpinPID(double moveSpeed, double targetAngle) {
    double spinOutput = -1.0*spinController.calculate(this.getAngle(), targetAngle);
    // double FeedForward;
    // if(Math.abs(this.getAngle()- targetAngle)>10){
    //   FeedForward = this.FeedForward;
    // }else{
    //   FeedForward = 0;
    // }
    if(spinOutput < 0){
      this.arcadeDrive(moveSpeed,spinOutput - FeedForward, false);
    }else{
      this.arcadeDrive(moveSpeed, spinOutput + FeedForward, false);
    }
  }

  public boolean atSpinPIDSetpoint(){
    return spinController.atSetpoint();
  }
  
  public void resetSpinPID(){
    spinController.reset();
  }
  
  /**
   * takes forward-backward and side to side values and drives the robot with them
   * @param moveValue forward-back speed, from -1 to 1, where 0 is stop
   * @param rotateValue side to side speed, from -1 to 1, where 0 is stop
   * @param squaredInputs whether or not inputs are sqaured, with sign retention
   */
  public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
    // local variables to hold the computed PWM values for the motors
    double leftMotorSpeed;
    double rightMotorSpeed;

    moveValue = limit(moveValue);

    rotateValue = limit(rotateValue);

    if (squaredInputs) {
      // square the inputs (while preserving the sign) to increase fine control
      // while permitting full power
      if (moveValue >= 0.0) {
        moveValue = moveValue * moveValue;
      } else {
        moveValue = -(moveValue * moveValue);
      }
      if (rotateValue >= 0.0) {
        rotateValue = rotateValue * rotateValue;
      } else {
        rotateValue = -(rotateValue * rotateValue);
      }
    }

    if (moveValue > 0.0) {
      if (rotateValue > 0.0) {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = Math.max(moveValue, rotateValue);
      } else {
        leftMotorSpeed = Math.max(moveValue, -rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      }
    } else {
      if (rotateValue > 0.0) {
        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      } else {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
      }
    }
    setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
  }

  public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
    if (leftMotorMaster == null || leftMotorFollower == null || rightMotorMaster == null
        || rightMotorFollower == null) {
      throw new NullPointerException("Null motor provided");
    }

    if(isReverse){
      if (leftMotorMaster != null && leftMotorFollower != null) {
        leftMotorMaster.set(limit(rightOutput*-1));
      }

      if (rightMotorMaster != null && rightMotorFollower != null) {
        rightMotorMaster.set(limit(leftOutput*-1));
      }
    }else{
      if (leftMotorMaster != null && leftMotorFollower != null) {
        leftMotorMaster.set(limit(leftOutput));
      }

      if (rightMotorMaster != null && rightMotorFollower != null) {
        rightMotorMaster.set(limit(rightOutput));
      }
    }
  }

  public void turnToAngle(double targetAngle, double turnSpeed) {
    double error = targetAngle - getAngle();
    if (error > 0) {
      // arcadeDrive(0, -1.0*(FeedForward + (kProportion * error)), false);
      setLeftRightMotorOutputs(1.0*(FeedForward + (kProportion * error)),
          -1.0*(FeedForward + (kProportion * error)) );
    } else {
      //arcadeDrive(0, -1.0*(-FeedForward + (kProportion * error)), false);
      setLeftRightMotorOutputs(1.0*(-FeedForward + (kProportion * error)),
        -1.0*(-FeedForward + (kProportion * error)) );
    }
  }

  public double getAngle() {
    // SmartDashboard.putNumber("gyro", imu.getYaw());
    return imu.getYaw();
  }

  public void zeroAngle() {
    imu.zeroYaw();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightVelocity(){
    if(isReverse){
      return leftEncoder.getVelocity()*-1;
    }else{
      return  rightEncoder.getVelocity();
    }
  }

  public double getLeftVelocity(){
    if(isReverse){
      return rightEncoder.getVelocity()*-1;
    }else{
      return  leftEncoder.getVelocity();
    }
  }

  public void resetLeftEncoder() {
    leftEncoder.setPosition(0.0);
    leftDriveEnc.reset();
  }

  public void resetRightEncoder() {
    rightEncoder.setPosition(0.0);
  }

  protected static double limit(double num) {
    if (num > 1.0) {
      return 1.0;
    }
    if (num < -1.0) {
      return -1.0;
    }
    return num;
  }

}
