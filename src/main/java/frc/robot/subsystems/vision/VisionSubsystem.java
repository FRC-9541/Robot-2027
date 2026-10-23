package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final VisionProcessor vision = new VisionProcessor();

    public VisionSubsystem() {
        
    }

    @Override
    public void periodic() {
        vision.calculateEstimatedPose2d();
    }

    public Pose2d getPoseFromVision() {
        return vision.getPose2d();
    }


}
    

