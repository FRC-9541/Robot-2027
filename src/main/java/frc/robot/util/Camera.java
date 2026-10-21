package frc.robot.util;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.CameraConstants.*;

/* util class to help with photon vision processing */
public class Camera {
	public static final PhotonCamera mainCamera = new PhotonCamera(MAIN_CAMERA_ID);
	public static PhotonPoseEstimator poseEstimator;
	public static Pose2d oldPose2d = new Pose2d(); // Initialize to avoid null pointers
	
	public static void init() {
		try {
			// 1. Load the official AprilTag layout for the current season
			AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(
			AprilTagFields.k2026RebuiltAndymark.m_resourceFile
			);
			
			// 2. Define camera offset relative to robot center (adjust in Constants)
			Transform3d robotToCam = new Transform3d(); 
			
			// 3. FIX: Modern constructor only takes 2 arguments
			poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
			
		} catch (IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), e.getStackTrace());
		}
	}
	
	/**
	* Returns the estimated 2D pose of the robot on the field.
	* If no target is visible, it returns the last known pose (oldPose2d).
	*/
	public static Pose2d getEstimatedPose2d() {
		if (poseEstimator == null) {
			return oldPose2d;
		}
		
		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		
		// Read all unread camera frames from the buffer
		for (var result : mainCamera.getAllUnreadResults()) {
			// Attempt the highly-accurate coprocessor multi-tag strategy first
			visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
			
			// Fall back to the lowest ambiguity single-tag strategy if multi-tag fails
			if (visionEst.isEmpty()) {
				visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
			}
			
			// If any strategy yields a valid pose, parse and update our saved pose
			if (visionEst.isPresent()) {
				oldPose2d = visionEst.get().estimatedPose.toPose2d();
			}
		}
		
		return oldPose2d;
	}
}
