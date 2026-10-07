package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoHandler {

    // TODO: 2027 doesn't support SendableChooser so change to Selectable
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final AutoFactory autoFactory;

    public AutoHandler(DriveSubsystem drive, LEDSubsystem led, ShooterSubsystem shooter) {
        this.autoFactory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drive // The drive subsystem
        );

        // auto options

        registerDefault("Do Nothing",
            Commands.none().withName("nothing"));

        register("Move 1 Meter Forward",
            drive.driveDistanceCommand(0.2, 1).withName("move1MeterForward"));

        register("TEST DON'T USE",
            simpleTrajCommand("NewPath"));

        register("test Choreo Command, moveForwardAndTurn", 
           simpleTrajCommand("moveForwardAndTurn"));

        SmartDashboard.putData("Auto Choices", autoChooser);
    }

    public Command getAutoRoutine() {
        return autoChooser.getSelected();
    }

    private Command simpleTrajCommand(String name) {
        return Commands.sequence(
            // MUST reset odometry before running a traj or else the robot will start from 0, 0 coords
            autoFactory.resetOdometry(name),
            autoFactory.trajectoryCmd(name)
        ).withName(name + "_Sequence");
    }

    private void register(String title, Command autoCommand) {
        autoChooser.addOption(title, autoCommand);
    } 

    private void registerDefault(String title, Command autoCommand) {
        autoChooser.setDefaultOption("Default: " + title, autoCommand);
    }
}
