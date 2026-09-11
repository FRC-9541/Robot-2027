// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;

/**
* The methods in this class are called automatically corresponding to each
* mode, as described in
* the TimedRobot documentation. If you change the name of this class or the
* package after creating
* this project, you must also update the Main.java file in the project.
*/
public class Robot extends TimedRobot {

	// autonomous Command that is ran at the start of autonomous
	private Command autoCommand;

	// robot code container
	private final RobotContainer robot = new RobotContainer();

	/*
	* This function is ran when the robot is first started up and should be used
	* for any
	* initialization code
	*/
	public Robot() {
		robot.configureBindings();
	}

	@Override
	public void autonomousInit() {
		autoCommand = robot.getAutonomousCommand();

    	if (autoCommand != null) {
      		autoCommand.schedule();
    	}
  	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
    	// teleop starts running. If you want the autonomous to
    	// continue until interrupted by another command, remove
    	// this line or comment it out.
    	if (autoCommand != null) {
      		autoCommand.cancel();
    	}
	} 
}
