package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final VisionProcessor vision = new VisionProcessor();
    private final Field2d field = new Field2d();

    public VisionSubsystem() {
        SmartDashboard.putData("Vision Field", field);
    }

    @Override
    public void periodic() {
        field.setRobotPose(vision.getPose2d());
        vision.calculateEstimatedPose2d();
    }

    public Pose2d getPoseFromVision() {
        return vision.getPose2d();
    }

}
