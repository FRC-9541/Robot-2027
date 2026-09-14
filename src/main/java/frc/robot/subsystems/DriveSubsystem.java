package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// all drive related objects and code should be contained in this class
public class DriveSubsystem extends SubsystemBase {

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
	// values given the drivetrain will stay until another value is given, so for
	// example if you set it to (0.7, 0.6) it will
	// keep driving until it get set to zero
	private final DifferentialDrive drivetrain = new DifferentialDrive(leftForwardDriveLead, rightForwardDriveLead);;

	// slew rate limiters stop a value from increasing more then the value set
	private SlewRateLimiter tankLeftFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
	private SlewRateLimiter tankRightFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
	private SlewRateLimiter arcadeFilter = new SlewRateLimiter(ARCADE_DRIVE_CONTROLLER_DAMPING);

	private final LTVUnicycleController controller = new LTVUnicycleController(0.02);

	public DriveSubsystem() {
		super();

		this.drivetrain.setMaxOutput(DRIVE_SCALE);
		
		// register driveConfig, values here can be finicky so blame this before others
		SparkMaxConfig driveConfig = new SparkMaxConfig();
		driveConfig.voltageCompensation(12); // 12 volt motors
		driveConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

		// guesses
		// TODO: move to constants
		double wheelDiameterMeters = 0.1524;
		double gearRatio = 8.45;
		double positionConversionFactor = (Math.PI * wheelDiameterMeters) / gearRatio;
		driveConfig.encoder.positionConversionFactor(positionConversionFactor);

		// TODO: split drive config into left and right for readability
		driveConfig.follow(leftForwardDriveLead);
		leftBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		driveConfig.follow(rightForwardDriveLead);
		rightBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		driveConfig.disableFollowerMode();
		driveConfig.inverted(false);
		leftForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		driveConfig.inverted(true);
		rightForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Set the distance per pulse for the drive encoders. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder
		// resolution.

		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);

		gyro.calibrate(); // takes a few seconds, make sure the robot is still
		gyro.reset();

		odometry = new DifferentialDriveOdometry(
				getRobotRotation2d(),
				leftEncoder.getPosition(),
				rightEncoder.getPosition(),
				new Pose2d(0.0, 0.0, new Rotation2d()) // sets starting tracking position to X:0, Y:0
		);
		// TODO: change dashboard in 2027
		SmartDashboard.putData("Field", field);
	}

	@Override
	public void periodic() {
		pose = odometry.update(getRobotRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

		// refresh monitoring metrics on the user dashboard
		field.setRobotPose(pose);
		// TODO: change dashboard
		SmartDashboard.putNumber("Odometry X (meters)", pose.getX());
		SmartDashboard.putNumber("Odometry Y (meters)", pose.getY());
		SmartDashboard.putNumber("Heading (degrees)", pose.getRotation().getDegrees());
	}

	private double calculateFilter(SlewRateLimiter filter, double value) {
		// if damping, return calculated, else return value
		return USE_DRIVE_DAMPING ? filter.calculate(value) : value;
	}

	public void followTrajectory(DifferentialSample sample) {
        // Get the velocity feedforward specified by the sample
        ChassisSpeeds ff = sample.getChassisSpeeds();

		DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0));

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = controller.calculate(
            pose,
            sample.getPose(),
            ff.vxMetersPerSecond,
            ff.omegaRadiansPerSecond
        );
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds); // 
        drivetrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

	public Pose2d getPose() {
		return pose;
	}

	public void resetOdometry(Pose2d newPose) {
		odometry.resetPose(newPose);
	}

	public Command driveWithControllerCommand(DoubleSupplier leftY, DoubleSupplier rightY, DoubleSupplier rightX, BooleanSupplier arcadeDrive) {
    	return run(() -> {
        	if (arcadeDrive.getAsBoolean()) {
            	drivetrain.arcadeDrive(calculateFilter(arcadeFilter, leftY.getAsDouble()), rightX.getAsDouble());
        	} else {
            	drivetrain.tankDrive(calculateFilter(tankLeftFilter, leftY.getAsDouble()), calculateFilter(tankRightFilter, rightY.getAsDouble()));
        	}
    	}).withName("driveWithController");
	}

	public Command driveDistanceCommand(double speed, double distanceMeters) {
		double left = leftEncoder.getPosition();
		double right = rightEncoder.getPosition();

		return arcadeDriveCommand(speed, 0)
			.until(() -> {
				return Math.max(leftEncoder.getPosition() - left, rightEncoder.getPosition() - right) >= distanceMeters;
			})
		
			.finallyDo(interrupted -> drivetrain.stopMotor()) // stop motor when it ends
		
			.withTimeout(Seconds.of(1)) // may need to get changed, on the safer side
			.withName("driveDistance");
	}

	public Command arcadeDriveCommand(double speed, double rotation) {
		return run(() -> drivetrain.arcadeDrive(speed, rotation))		
		.withName("arcadeDrive");
	}

	// tank drive is where one value controls one side of the movement, and the
	// other controls the other
	public Command tankDriveCommand(double left, double right) {
		return run(() -> drivetrain.tankDrive(left, right)).withName("tankDrive");
	}

	// stops the drivetrain from driving
	// cy says this comment is not needed, I think otherwise
	public Command stopDriveCommand() {
		return runOnce(() -> drivetrain.stopMotor()).withName("stopDrive");
	}

	public Rotation2d getRobotRotation2d() {
		return Rotation2d.fromDegrees(-gyro.getAngle());
	}
}
