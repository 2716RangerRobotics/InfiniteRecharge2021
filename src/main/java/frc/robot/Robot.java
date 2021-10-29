/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.oldauto.AutoAttackAtAngle;
import frc.robot.commands.oldauto.AutoDriveBackwards;
import frc.robot.commands.oldauto.AutoDriveTurnAndScore;
import frc.robot.commands.oldauto.AutoFakeSneakAttack;
import frc.robot.commands.oldauto.AutoFakeTwoBallTake;
import frc.robot.commands.oldauto.AutoDriveStraight;
import frc.robot.commands.oldauto.AutoDriveStraightAndScore;
import frc.robot.commands.oldauto.AutoDriveToRendezvousAndScore;
import frc.robot.commands.oldauto.AutoFeedShooter;
import frc.robot.commands.oldauto.AutoFeederStationPosition;
import frc.robot.commands.oldauto.AutoLetThemEatBalls;
import frc.robot.commands.AutoBarrelPath;
import frc.robot.commands.AutoDriveAndShoot;
import frc.robot.commands.AutoGift340;
import frc.robot.commands.AutoSlalomSimple;
import frc.robot.commands.BarrelPath;
import frc.robot.commands.BouncePath;
import frc.robot.commands.oldauto.AutoSneakAttack;
import frc.robot.commands.oldauto.AutoSpitAndTurn;
import frc.robot.commands.oldauto.AutoTwoBallTake;
import frc.robot.commands.DriveBrakeOn;
import frc.robot.commands.DriveCoastOn;
import frc.robot.commands.DrivePathWeaver;
import frc.robot.commands.SlalomPath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public Command autonomousCommand;

  public RobotContainer robotContainer;
  SendableChooser<Command> chooser = new SendableChooser<>();

//}
// Robot extends IterativeRobot {

//   DigitalInput gearSwitch;
// }
// private DigitalInput gearSensor  = new DigitalInput(Constants.GEAR_SENSOR_DIO_ID);


// 	public boolean isGearPresent() {
// 		return !gearSensor.get();
// 	}
	
// 	public void updateStatus(Robot.OperationMode operationMode) {
// 		SmartDashboard.putBoolean("Gear Sensor", isGearPresent());
// 		if (isGearPresent()) {
//       for (int isGearPresent = ; isGearPresent <0 i <6; isGearPresent++) {
//       System.out.println("Balls in ball inatke #" + isGearPresent);
//     }
//   }
// 		else {
//         System.out.println("Balls in ball inatke #" + isGearPresent);
// 		}
//   }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    //gearSwitch = new DigitalInput(0);
    //imu = new AHRS(Port.kMXP);

    //CommandBase.init();
    
    // chooser.addOption("SlalomPath", new SlalomPath());
    // chooser.addOption("BouncePath", new BouncePath());
    // chooser.addOption("BarrelPath", new BarrelPath());
    // chooser.addOption("DriveStraight", new DrivePathWeaver("DriveStraight", false));
    // chooser.addOption("CoolerBarrelPath", new AutoBarrelPath());
    chooser.addOption("AutoDriveAndShoot", new AutoDriveAndShoot());
    chooser.addOption("AutoDriveStraight", new AutoDriveStraight());
    //chooser.addOption("AutoDriveBackwards", new AutoDriveBackwards());
    //chooser.addOption("AutoDriveAndScore ~13ft~", new AutoDriveTurnAndScore()); 
    chooser.addOption("AutoDriveStraightAndScore", new AutoDriveStraightAndScore());
    chooser.addOption("AutoLetThemEatBalls", new AutoLetThemEatBalls());
    chooser.addOption("AutoGift340", new AutoGift340());
    // chooser.addOption("AutoFeederStationPosition", new AutoFeederStationPosition());
    // chooser.addOption("AutoFeedShooter", new AutoFeedShooter());
    // chooser.addOption("AutoTwoBallTake", new AutoTwoBallTake());
    // chooser.addOption("AutoFakeTwoBallTake", new AutoFakeTwoBallTake());
    // chooser.addOption("AutoSneakAttack", new AutoSneakAttack());
    // chooser.addOption("AutoFakeSneakAttack", new AutoFakeSneakAttack());
    // chooser.addOption("AutoDriveToRendezvousAndScore", new AutoDriveToRendezvousAndScore());
    // chooser.addOption("AutoSpitAndTurn", new AutoSpitAndTurn());
    // chooser.addOption("AutoAttackAtAngle", new AutoAttackAtAngle());
    // chooser.addOption("AutoSlalomSimple", new AutoSlalomSimple());
		chooser.addOption("None", null);

		SmartDashboard.putData("Auto Mode", chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(new DriveCoastOn());
  }

  @Override
  public void disabledPeriodic() {
    Command autoModeToRun = chooser.getSelected();
    if(autoModeToRun != null){
      SmartDashboard.putString("Auto Mode To Run", autoModeToRun.getName());
    }else{
      SmartDashboard.putString("Auto Mode To Run", "NO AUTO TO RUN!");
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
      //startRecording();

		autonomousCommand = chooser.getSelected();

		//resetIMU();

		if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    // new DrivePathWeaver("DriveStraight", false).schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().schedule(new DriveBrakeOn());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}