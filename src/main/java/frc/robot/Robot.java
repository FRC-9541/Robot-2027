// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drive.DriveTrain;
import frc.robot.lighting.LEDController;

import static frc.robot.Constants.OperatingConstants.*;

/**
* The methods in this class are called automatically corresponding to each
* mode, as described in
* the TimedRobot documentation. If you change the name of this class or the
* package after creating
* this project, you must also update the Main.java file in the project.
*/
public class Robot extends TimedRobot {
	
	private final DriveTrain driveTrain;
	private final LEDController ledController;
	
	// TODO: 2027 doesn't support SendableChooser so find alternatives when upgrading
	// TODO: also clean chooser code code to be more readable
	private static final String TEST_A = "A";
	private static final String TEST_B = "B";

	private String testSelected;
	private final SendableChooser<String> chooser = new SendableChooser<>();
	
	// actual controllers use by driver and operator, can't be accessed during disabled
	private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
	private final XboxController opController = new XboxController(OPERATOR_CONTROLLER_PORT);
	
	/*
	* This function is ran when the robot is first started up and should be used
	* for any
	* initialization code
	*/
	public Robot() {
		CameraServer.startAutomaticCapture();
		
		// instantiate the driveTrain
		// TODO: have control mode actually match chooser, right now the value just gets set
		driveTrain = new DriveTrain(false);

		ledController = new LEDController(92);
		
		chooser.setDefaultOption("Default: value A", TEST_A);
		chooser.addOption("Test: value B", TEST_B);

		SmartDashboard.putData("Test choices", chooser);
	}
	
	/**
	* This function is called every 20 ms, no matter the mode. Use this for items
	* like diagnostics
	* that you want ran during disabled, autonomous, teleoperated and test.
	*
	* This runs after the mode specific periodic functions, but before LiveWindow
	* and
	* SmartDashboard integrated updating.
	*/
	@Override
	public void robotPeriodic() {
		driveTrain.robotPeriodic();
	}
	
	/**
	* This autonomous (along with the chooser code above) shows how to select
	* between different
	* autonomous modes using the dashboard. The sendable chooser code works with
	* the Java
	* SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
	* chooser code and
	* uncomment the getString line to get the auto name from the text box below the
	* Gyro
	*
	* You can add additional auto modes by adding additional comparisons to the
	* switch structure
	* below with additional strings. If using the SendableChooser make sure to add
	* them to the
	* chooser code above as well.
	*/
	@Override
	public void autonomousInit() {
		updateSelected();
	}
	
	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		ledController.updateLEDS();
	}
	
	/* This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		updateSelected();
	}
	
	/* This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {

		// slow mode lets the driver control the bot easier by slowing down the max speed of the bot
		boolean slowMode = driverController.getLeftBumperButton();
		
		// drive with controller
		driveTrain.controllerDrive(driverController.getLeftY(), driverController.getRightY(), slowMode);
	}
	
	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
	}
	
	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {
		updateSelected();
		ledController.updateLEDS();
	}
	
	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {
	}
	
	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}
	
	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}
	
	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
	
	public void updateSelected() { // Only run when starting a mode, not during.
		testSelected = chooser.getSelected();
		System.out.print(testSelected);
	}
}
