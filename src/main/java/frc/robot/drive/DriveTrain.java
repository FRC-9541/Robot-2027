package frc.robot.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Constants.DriveConstants.*;


// all drive related objects and code should be contained in this class for ease of changing
public class DriveTrain {

    // get SparkMaxes 
	// TODO: rename SparkMaxes to have better naming scheme
    private final SparkMax leftForwardDriveLead = new SparkMax(LEFT_FORWARD_ID, MotorType.kBrushed);
    private final SparkMax leftBackDriveFollower = new SparkMax(LEFT_BACK_ID, MotorType.kBrushed);
    private final SparkMax rightForwardDriveLead = new SparkMax(RIGHT_FORWARD_ID, MotorType.kBrushed);
    private final SparkMax rightBackDriveFollower = new SparkMax(RIGHT_BACK_ID, MotorType.kBrushed);

	// will be controlling the movement of the bot
	// values given the driveTrain will stay until another value is given, so for example if you set it to (0.7, 0.6) it will 
	// keep driving until it get set to zero
    private final DifferentialDrive driveTrain = new DifferentialDrive(leftForwardDriveLead, rightForwardDriveLead);

    // slew rate limiters stop a value from increasing more then the value set
    private SlewRateLimiter tankLeftFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
    private SlewRateLimiter tankRightFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
    private SlewRateLimiter arcadeFilter = new SlewRateLimiter(ARCADE_DRIVE_CONTROLLER_DAMPING);

	// true = arcade, false = tank
	private boolean driveMode;

    public DriveTrain(boolean driveMode) {
		this.driveMode = driveMode;

		// register driveConfig, values here can be finicky so blame this before others
		SparkMaxConfig driveConfig = new SparkMaxConfig();
		driveConfig.voltageCompensation(12); // 12 volt motors
		driveConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
		
		driveConfig.follow(leftForwardDriveLead);
		leftBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		driveConfig.follow(rightForwardDriveLead);
		rightBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		
		driveConfig.disableFollowerMode();
		driveConfig.inverted(false);
		leftForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		driveConfig.inverted(true);
		rightForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


	// drive train gets controlled by the controller and also has slew filters
	// if slowMode is true, it drives slower
	public void controllerDrive(double leftY, double rightY, boolean slowMode) {
		double driveScale;
		double rotateScale;

		if (slowMode) {
			driveScale = SLOW_DRIVE_SCALE;
			rotateScale = SLOW_ROTATION_SCALE;
		} else {
			driveScale = DRIVE_SCALE;
			rotateScale = DRIVE_SCALE;
		}
		
		if (driveMode) { // true = arcade, false = tank
			arcadeDrive(calculateFilter(arcadeFilter, leftY) * driveScale, rightY * rotateScale);
		} else {
			tankDrive(calculateFilter(tankLeftFilter, leftY) * driveScale, calculateFilter(tankRightFilter, rightY) * driveScale);
		}
		
	}

	public double calculateFilter(SlewRateLimiter filter, double value) {

		// if damping, calculate, else  
		return USE_DRIVE_DAMPING ? filter.calculate(value) : value;
	}

	// tank drive is where one value controls one side of the movement, and the other controls the other
	public void tankDrive(double left, double right) {
		driveTrain.tankDrive(left, right);
	}

	// stops the tankDrive from driving
	public void stopTankDrive() {
		driveTrain.tankDrive(0, 0);
	}

	public void arcadeDrive(double speed, double rotation) {
		driveTrain.arcadeDrive(speed, rotation);
	}

	public void setDriveMode(boolean value) {
		driveMode = value;
	}
    
}
