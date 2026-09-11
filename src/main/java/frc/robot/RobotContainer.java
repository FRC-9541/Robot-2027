package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import static frc.robot.Constants.OperatingConstants.*;

public class RobotContainer {


    // subsystems
    private final DriveSubsystem drive = new DriveSubsystem();
    private final LEDSubsystem led = new LEDSubsystem();

    // TODO: 2027 doesn't support SendableChooser so find alternatives when upgrading
	private static final String TEST_A = "A";
	private static final String TEST_B = "B";

	private String testSelected;
	private final SendableChooser<String> chooser = new SendableChooser<>();
	
	// actual controllers use by driver and operator, can't be accessed during disabled
	private final XboxController driver = new XboxController(DRIVER_CONTROLLER_PORT);
	private final XboxController operator = new XboxController(OPERATOR_CONTROLLER_PORT);
    
    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        chooser.setDefaultOption("Default: value A", TEST_A);
		chooser.addOption("value B", TEST_B);

		SmartDashboard.putData("Test choices", chooser);
    }

    public void updateSelected() { // only run when starting a mode, not during.
		testSelected = chooser.getSelected();
		System.out.print(testSelected);
	}

    public void configureBindings() {
        drive.setDefaultCommand(
            drive.driveWithControllerCommand(driver.getLeftY(), driver.getRightY(), driver.getRightX(), true)
        ); 
    }

    public Command getAutonomousCommand() {
        // TODO: replace with actual Auto Commands (from chooser)
        return new Command() {};
    }

}