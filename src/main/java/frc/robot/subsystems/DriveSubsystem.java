package frc.robot.subsystems;

import com.revrobotics.PersistMode;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.DrivetrainConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/* all drive related objects and code should be contained in this class */
public class DriveSubsystem extends SubsystemBase {

	// NOTE: lead spark maxes are the towards the front of the robot, and followers are to the back
	private final SparkMax leftLeadMotor = new SparkMax(LEFT_FORWARD_ID, MotorType.kBrushed);
	private final SparkMax leftFollowMotor = new SparkMax(LEFT_BACK_ID, MotorType.kBrushed);
	private final SparkMax rightLeadMotor = new SparkMax(RIGHT_FORWARD_ID, MotorType.kBrushed);
	private final SparkMax rightFollowMotor = new SparkMax(RIGHT_BACK_ID, MotorType.kBrushed);

	private final ADIS16470_IMU gyro = new ADIS16470_IMU();

	// will be controlling the movement of the bot
	// keeps driving until it gets set to zero
	private final DifferentialDrive drivetrain = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
	
	private final LTVUnicycleController controller = new LTVUnicycleController(0.02);
	private final DifferentialDriveOdometry odometry;
	private final Field2d field = new Field2d();
	private Pose2d pose = new Pose2d();


	// slew rate limiters stop a value from increasing more then the value set
	private SlewRateLimiter tankLeftFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
	private SlewRateLimiter tankRightFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
	private SlewRateLimiter arcadeFilter = new SlewRateLimiter(ARCADE_DRIVE_CONTROLLER_DAMPING);


	public DriveSubsystem() {
		configDrive();

		leftLeadMotor.getEncoder().setPosition(0);
		rightLeadMotor.getEncoder().setPosition(0);

		gyro.calibrate(); // takes a few seconds, make sure the robot is still
		gyro.reset();

		odometry = new DifferentialDriveOdometry(
				getRobotRotation2d(),
				leftLeadMotor.getEncoder().getPosition(),
				rightLeadMotor.getEncoder().getPosition(),
				new Pose2d(0.0, 0.0, new Rotation2d()) // sets starting tracking position to X:0, Y:0
		);
		// TODO: change dashboard in 2027
		SmartDashboard.putData("Field", field);
	}

	@Override
	public void periodic() {
		pose = odometry.update(getRobotRotation2d(), leftLeadMotor.getEncoder().getPosition(), rightLeadMotor.getEncoder().getPosition());

		// refresh monitoring metrics on the user dashboard
		field.setRobotPose(pose);
	}

	public void configDrive() {
		// scale all movement so robot is capped at a certain max speed for safety
		this.drivetrain.setMaxOutput(DRIVE_SCALE);
		
		SparkMaxConfig globalConfig = new SparkMaxConfig();
		globalConfig.voltageCompensation(DRIVE_MOTOR_VOLTAGE)
			.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
			.encoder.positionConversionFactor(POSITION_FACTOR);

		
		SparkMaxConfig leftLeadConfig = new SparkMaxConfig().apply(globalConfig);
		SparkMaxConfig leftFollowConfig = new SparkMaxConfig().apply(globalConfig);
		leftLeadConfig.inverted(false);
		leftFollowConfig.follow(leftLeadMotor);

		leftLeadMotor.configure(leftLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		leftFollowMotor.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkMaxConfig rightLeadConfig = new SparkMaxConfig().apply(globalConfig);
		SparkMaxConfig rightFollowConfig = new SparkMaxConfig().apply(globalConfig);
		rightLeadConfig.inverted(true);
		rightFollowConfig.follow(rightLeadMotor);

		rightLeadMotor.configure(rightLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightFollowMotor.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	private double calculateFilter(SlewRateLimiter filter, double value) {
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
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
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
            	drivetrain.arcadeDrive(calculateFilter(arcadeFilter, -leftY.getAsDouble()), -rightX.getAsDouble());
        	} else {
            	drivetrain.tankDrive(calculateFilter(tankLeftFilter, -leftY.getAsDouble()), calculateFilter(tankRightFilter, -rightY.getAsDouble()));
        	}
    	}).withName("driveWithController");
	}

	// TODO: not functioning, could be a issue that has now been fixed, but make sure
	public Command driveDistanceCommand(double speed, Distance distance) {
		double leftSnapshot = leftLeadMotor.getEncoder().getPosition();
		double rightSnapshot = rightLeadMotor.getEncoder().getPosition();

		return arcadeDriveCommand(speed, 0)
			.until(() -> {
				return Math.max(leftLeadMotor.getEncoder().getPosition() - leftSnapshot, rightLeadMotor.getEncoder().getPosition() - rightSnapshot) >= distance.abs(Meters);
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
