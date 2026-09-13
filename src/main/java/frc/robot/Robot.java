// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

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
	public void robotPeriodic() {
		// for the commands to work
    	CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		Elastic.selectTab(1);
		autoCommand = robot.getAutonomousCommand();
		Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Starting Auto", "Running Auto Command: " + (autoCommand == null ? "Null" : autoCommand.getName()), 5000));


		// starts auto command if it exists
    	if (autoCommand != null) {
      		CommandScheduler.getInstance().schedule(autoCommand);
    	}
  	}

	@Override
	public void teleopInit() {
		Elastic.selectTab(0);
		// stops the auto command at the start of teleop so we can control
    	if (autoCommand != null) {
      		autoCommand.cancel();
    	}
	} 
}
