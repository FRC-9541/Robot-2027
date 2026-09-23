package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Elastic;

import static frc.robot.Constants.OperatingConstants.*;

import choreo.auto.AutoFactory;

public class RobotContainer {

    // subsystems
    private final DriveSubsystem drive = new DriveSubsystem();
    private final LEDSubsystem led = new LEDSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    // TODO: 2027 doesn't support SendableChooser so change to Selectable
	private final SendableChooser<Command> testChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> driveChooser = new SendableChooser<>();

	// actual controllers use by driver and operator, can't be accessed during disabled
	private final CommandXboxController driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
	private final XboxController operator = new XboxController(OPERATOR_CONTROLLER_PORT);

    private final AutoFactory autoFactory;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        autoFactory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drive // The drive subsystem
        );

        autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("nothing"));
        autoChooser.addOption("Move 1 Meter Forward", drive.driveDistanceCommand(0.2, 1).withName("move1MeterForward"));
        autoChooser.addOption("test Choreo Command, moveForwardAndTurn", moveForwardAndTurnAuto());
        SmartDashboard.putData("Auto Choices", autoChooser);

        driveChooser.setDefaultOption("Default: Arcade Drive", true);
		driveChooser.addOption("Tank Drive", false);
        SmartDashboard.putData("Drive Choices", driveChooser);
    }

    // TODO: move to another class
    public Command moveForwardAndTurnAuto() {
        return autoFactory.trajectoryCmd("moveForwardAndTurn").withName("moveForwardAndTurn");
    }

    public void configureBindings() {
        drive.setDefaultCommand(
            drive.driveWithControllerCommand(driver::getLeftY, driver::getRightY, driver::getRightX, driveChooser::getSelected)
        ); 

        shooter.setDefaultCommand(
            shooter.launchCommand(operator.getRightBumperButton(), operator.getRightBumperButtonPressed(), operator.getAButton(), operator.getYButton())
        );

        Elastic.sendNotification(new Elastic.Notification(Elastic.NotificationLevel.INFO, "robot start finished", "yippie"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}