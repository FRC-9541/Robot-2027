package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoHandler;
import frc.robot.lighting.LEDPatterns;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.OperatingConstants.*;

public class RobotContainer {

    // subsystems
    private final DriveSubsystem drive = new DriveSubsystem();
    private final LEDSubsystem led = new LEDSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private final AutoHandler autoHandler = new AutoHandler(drive, led, shooter);

    // TODO: 2027 doesn't support SendableChooser so change to Selectable
    private final SendableChooser<Boolean> driveChooser = new SendableChooser<>();

	// actual controllers use by driver and operator, can't be accessed during disabled
	private final CommandXboxController driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
	private final XboxController operator = new XboxController(OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        CameraServer.startAutomaticCapture();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        driveChooser.setDefaultOption("Default: Arcade Drive", true);
		driveChooser.addOption("Tank Drive", false);
        SmartDashboard.putData("Drive Choices", driveChooser);
    }

    public void configureBindings() { // configure command bindings
        drive.setDefaultCommand(
            drive.driveWithControllerCommand(driver::getLeftY, driver::getRightY, driver::getRightX, driveChooser::getSelected)
        ); 

        shooter.setDefaultCommand(
            shooter.launchCommand(operator.getRightBumperButton(), operator.getRightBumperButtonPressed(), operator.getAButton(), operator.getYButton())
        );

        led.setDefaultCommand(
            led.updateLEDsCommand()
        );

        new Trigger(RobotState::isTeleop).onTrue(led.setCatcherPatternCommand(LEDPatterns.GOLD_BLUE_CHASE));
    }

    public Command getAutonomousCommand() {
        return autoHandler.getAutoRoutine();
    }
}