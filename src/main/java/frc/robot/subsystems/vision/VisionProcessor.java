package frc.robot.subsystems.vision;

import static frc.robot.Constants.CameraConstants.MAIN_CAMERA_ID;
import static frc.robot.Constants.CameraConstants.MAIN_CAMERA_LOCATION;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/* class to help with photon vision processing */
public class VisionProcessor {
	public final PhotonCamera mainCamera = new PhotonCamera(MAIN_CAMERA_ID);
	public PhotonPoseEstimator poseEstimator;
	public Pose2d estPose2d = new Pose2d();

	public VisionProcessor() {
		try {
			AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(
					AprilTagFields.k2026RebuiltAndymark.m_resourceFile);

			poseEstimator = new PhotonPoseEstimator(fieldLayout, MAIN_CAMERA_LOCATION);

		} catch (IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), e.getStackTrace());
		}
	}

	/**
	 * Calculate the estimated pose gotten from the camera.
	 * It should get called periodicity to be accurate.
	 * Pose can be gotten from {@link #getPose2d()}.
	 */
	public void calculateEstimatedPose2d() {
		if (poseEstimator == null)
			return;

		Optional<EstimatedRobotPose> visionEst = Optional.empty();

		for (var result : mainCamera.getAllUnreadResults()) {
			visionEst = poseEstimator.estimateCoprocMultiTagPose(result);

			if (visionEst.isEmpty()) {
				visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
			}

			if (visionEst.isPresent()) {
				estPose2d = visionEst.get().estimatedPose.toPose2d();
			}
		}
	}

	/**
	 * Pose that the camera sees. 
	 * Only updates with {@link #calculateEstimatedPose2d()}.
	 * @return Pose2d of the robot.
	 */
	public Pose2d getPose2d() {
		return estPose2d;
	}
}
