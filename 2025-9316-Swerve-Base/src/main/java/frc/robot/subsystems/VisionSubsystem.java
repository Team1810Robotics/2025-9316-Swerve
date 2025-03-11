package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult result;
    
    // PID controllers for different axes of movement
    public PIDController driveControllerY = new PIDController(VisionConstants.VY_Kp, VisionConstants.VY_Ki, VisionConstants.VY_Kd);
    public PIDController driveControllerX = new PIDController(VisionConstants.VX_Kp, VisionConstants.VX_Ki, VisionConstants.VX_Kd);

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    
    // Transform from camera to robot center (measurements in meters)
    public static final Transform3d CAMERA_TO_ROBOT = 
        new Transform3d(new Translation3d(.05, 0.0, .15), new Rotation3d(0, 0, 0));
    
    public VisionSubsystem() {
        // Initialize camera with name from constants
        camera = new PhotonCamera(VisionConstants.TARGET_CAMERA);
        
        // Initialize pose estimator with field layout and camera transform
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                CAMERA_TO_ROBOT);
        
        // Get initial result
        result = camera.getLatestResult();

        // Add debug info to Shuffleboard
        Shuffleboard.getTab("Vision").addBoolean("Camera Connected", () -> camera.isConnected());
        Shuffleboard.getTab("Vision").addBoolean("Has Target", () -> hasTarget());
        Shuffleboard.getTab("Vision").addNumber("Target Yaw", () -> getYaw().orElse(0.0));
        Shuffleboard.getTab("Vision").addNumber("Target Range", () -> getRange().orElse(0.0));
        
        // Add PIDs to Shuffleboard for tuning
        Shuffleboard.getTab("Vision").add("Y PID", driveControllerY);
        Shuffleboard.getTab("Vision").add("X PID", driveControllerX);
    }

    @Override
    public void periodic() {
        // Update the latest result from the camera
        result = camera.getLatestResult();
    }

    /**
     * Gets the estimated robot pose based on vision
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(result);
    }

    /**
     * Checks if the camera sees any AprilTags
     */
    public boolean hasTarget() {
        return result.hasTargets();
    }

    /**
     * Gets the yaw (horizontal angle) to the best target
     */
    public Optional<Double> getYaw() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getYaw());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Gets the range (distance in meters) to the best target
     */
    public Optional<Double> getRange() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getX());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Gets the horizontal offset (Y in meters) to the best target
     */
    public Optional<Double> getHorizontalOffset() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getY());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Calculates the forward/backward power needed to reach desired distance from target
     * 
     * @param defaultPower Value to use when no target is visible
     * @param targetDistance Desired distance from target in meters
     * @param enableVision Whether to use vision for alignment
     * @return The calculated X power
     */
    public double calculateXPower(double defaultPower, double targetDistance, boolean enableVision) {
        if (hasTarget() && enableVision) {
            return driveControllerX.calculate(getRange().get(), targetDistance);
        } else {
            return defaultPower;
        }
    }

    /**
     * Calculates the left/right power needed to center with the target
     * 
     * @param defaultPower Value to use when no target is visible
     * @param targetOffset Desired horizontal offset (usually 0)
     * @param enableVision Whether to use vision for alignment
     * @return The calculated Y power
     */
    public double calculateYPower(double defaultPower, double targetOffset, boolean enableVision) {
        if (hasTarget() && enableVision) {
            // Use the yaw for centering - negative because positive yaw means target is to the right
            return driveControllerY.calculate(getYaw().get(), targetOffset);
        } else {
            return defaultPower;
        }
    }
}