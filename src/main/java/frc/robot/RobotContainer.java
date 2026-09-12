package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lighting.LEDPatterns;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import static frc.robot.Constants.OperatingConstants.*;

import java.util.Set;

public class RobotContainer {


    // subsystems
    private final DriveSubsystem drive = new DriveSubsystem();
    private final LEDSubsystem led = new LEDSubsystem();

    // TODO: 2027 doesn't support SendableChooser so change to Selectable
	private final SendableChooser<Command> testChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> driveChooser = new SendableChooser<>();

	
	// actual controllers use by driver and operator, can't be accessed during disabled
	private final CommandXboxController driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
	private final CommandXboxController operator = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    
    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        // set commands to chooser
        testChooser.setDefaultOption("Default: Red", led.setCatcherPatternCommand(LEDPatterns.RED));
		testChooser.addOption("Green", led.setCatcherPatternCommand(LEDPatterns.GREEN));
        SmartDashboard.putData("Test choices", testChooser);

        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive 1 Meter Forward", drive.driveDistanceCommand(0.2, 1));
        SmartDashboard.putData("Auto Choices", autoChooser);

        driveChooser.setDefaultOption("Default: Arcade Drive", true);
		driveChooser.addOption("Tank Drive", false);
        SmartDashboard.putData("Drive choices", driveChooser);
		
    }

    public void configureBindings() {
        drive.setDefaultCommand(
            drive.driveWithControllerCommand(driver.getLeftY(), driver.getRightY(), driver.getRightX(), driveChooser::getSelected)
        ); 

        // example code for led subsystem, get set to either red or green if a is pressed, or blue if b is pressed
        driver.a().onTrue(Commands.defer(testChooser::getSelected, Set.of(led)));
        driver.b().onTrue(led.setCatcherPatternCommand(LEDPatterns.BLUE));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}