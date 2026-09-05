package frc.robot.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.DriveConstants.*;


// all drive related objects and code should be contained in this class for ease of changing
public class DriveTrain {
	
	// get SparkMaxes 
	// TODO: rename spark maxes to have better naming scheme
	private final SparkMax leftForwardDriveLead = new SparkMax(LEFT_FORWARD_ID, MotorType.kBrushed);
	private final SparkMax leftBackDriveFollower = new SparkMax(LEFT_BACK_ID, MotorType.kBrushed);
	private final SparkMax rightForwardDriveLead = new SparkMax(RIGHT_FORWARD_ID, MotorType.kBrushed);
	private final SparkMax rightBackDriveFollower = new SparkMax(RIGHT_BACK_ID, MotorType.kBrushed);
	
	// get encoders from spark maxes
	private final RelativeEncoder leftEncoder = leftForwardDriveLead.getEncoder();
	private final RelativeEncoder rightEncoder = rightForwardDriveLead.getEncoder();
	
	// get gyro
	private final ADIS16470_IMU gyro = new ADIS16470_IMU();
	
	private final DifferentialDriveOdometry odometry;
	private Pose2d pose = new Pose2d();
  	private final Field2d field = new Field2d();
	
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

		double wheelDiameterMeters = 0.1524; 
  		double gearRatio = 8.45;
  		double positionConversionFactor = (Math.PI * wheelDiameterMeters) / gearRatio;

		driveConfig.encoder.positionConversionFactor(positionConversionFactor);
		
		// Set the distance per pulse for the drive encoders. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder
		// resolution.

		
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);

		gyro.calibrate(); // Takes a few seconds, make sure the robot is still
  		gyro.reset();  
		
		odometry = new DifferentialDriveOdometry(
        	getRobotRotation2d(),
        	leftEncoder.getPosition(),
        	rightEncoder.getPosition(),
        	new Pose2d(0.0, 0.0, new Rotation2d()) // Sets starting tracking position to X:0, Y:0
    	);


		SmartDashboard.putData("Field", field);
	}

	private Rotation2d getRobotRotation2d() {
    	return Rotation2d.fromDegrees(-gyro.getAngle());
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
	
	// stops the driveTrain from driving
	public void stopDrive() {
		driveTrain.tankDrive(0, 0);
	}
	
	public void arcadeDrive(double speed, double rotation) {
		driveTrain.arcadeDrive(speed, rotation);
	}
	
	public void setDriveMode(boolean value) {
		driveMode = value;
	}

	public void robotPeriodic() {
		pose = odometry.update(
        	getRobotRotation2d(),
        	leftEncoder.getPosition(),
        	rightEncoder.getPosition()
    );

    // Refresh monitoring metrics on the user dashboard
    field.setRobotPose(pose);
    SmartDashboard.putNumber("Odometry X (meters)", pose.getX());
    SmartDashboard.putNumber("Odometry Y (meters)", pose.getY());
    SmartDashboard.putNumber("Heading (degrees)", pose.getRotation().getDegrees());
	}
	
}
