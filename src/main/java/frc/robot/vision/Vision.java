package frc.robot.vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/** 
 * Used for fetching data from Vision APIs.
 * 
 * Designed with Limelight in mind, but could be refactored for use with Photonlib since that is
 * already used for simulation.
 */
public class Vision {
    private VisionSim visionSim;

    public Vision() {
        if (Robot.isSimulation()) {
            visionSim = new VisionSim();
        }
        var cameraPos = kCameraPos;
        LimelightHelpers.setCameraPose_RobotSpace(
            kCameraName,
            cameraPos.getX(),
            cameraPos.getY(),
            cameraPos.getZ(),
            cameraPos.getRotation().getX(),
            cameraPos.getRotation().getY(),
            cameraPos.getRotation().getZ()
        );
    }

    /** 
     * Update Drivesubsystem's position (pose estimator) using vision. Auto rejects measurements
     * deemed to be unreliable. 
     */
    public void update(DriveSubsystem driveSubsystem) {
        if (Robot.isSimulation()) {
            updateSimulation(driveSubsystem);
            return;
        }
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            driveSubsystem.getPose().getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );
        PoseEstimate estimate = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // strategy taken from: https://www.chiefdelphi.com/t/limelight-odometry-question/433311/6
        if (driveSubsystem.shouldAcceptVisionMeasurement(estimate)) {
            double poseDifference = driveSubsystem.getPose()
                                                  .getTranslation()
                                                  .getDistance(estimate.pose.getTranslation());
            double xyStdDevs = .7;
            double degStdDevs = 9999999;
            // multiple visible tags
            if (estimate.tagCount >= 2) {
                xyStdDevs = .5;
                degStdDevs = 6;
            } 
            // target has large area and estimated pose is close
            else if (getBestTargetArea(estimate) > 0.8 && poseDifference < 0.5) {
                xyStdDevs = 1;
                degStdDevs = 12;
            }
            // target is further away, but estimated pose is closer
            else if (getBestTargetArea(estimate) > 0.1 && poseDifference < 0.3) {
                xyStdDevs = 2;
                degStdDevs = 30;
            }
            driveSubsystem.addVisionMeasurement(
                estimate.pose, estimate.timestampSeconds,
                VecBuilder.fill(xyStdDevs, xyStdDevs, Units.degreesToRadians(degStdDevs)));
        }
    }

    /** Update the vision simulation. */
    private void updateSimulation(DriveSubsystem driveSubsystem) {
        if (visionSim == null) {
            DriverStation.reportError("Vision.updateSimulation(): Vision Simulation is enabled but not instantiated. Doing nothing", null);
            return;
        }
        var estPose = visionSim.getEstimatedRobotPose();
        estPose.ifPresent(est -> {
            var estStdDevs = visionSim.getEstimationStdDevs();
            driveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

        visionSim.updateSimulation(driveSubsystem.getPose());

        var debugField = visionSim.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(driveSubsystem.getPose());
    }

    // --- Utility --------------------------------------------------------------------------------

    /** Returns the best tag area (tag that appears the closest) from a vision estimate. */
    public static double getBestTargetArea(PoseEstimate estimate) {
        if (estimate.rawFiducials.length == 0) {
            return 0;
        }
        double max = estimate.rawFiducials[0].ta;
        for (var f : estimate.rawFiducials) {
            if (f.ta > max) {
                max = f.ta;
            }
        }
        return max;
    }

    // --- Simulation -----------------------------------------------------------------------------

    /** Class that handles the simulated vision pipeline. */
    private class VisionSim {
        private VisionSystemSim visionSim = new VisionSystemSim("visionSim");
        private PhotonCamera camera = new PhotonCamera("photonLimelight");
        // basically a placeholder for sim since the limelight firmware and api is used on the real
        // robot
        private SimCameraProperties cameraProperties = new SimCameraProperties();
        private PhotonCameraSim cameraSim;
        private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kCameraPos
        );
        private Matrix<N3, N1> curStdDevs;

        private VisionSim() {
            // add all simulated april tag targets
            visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));
            // TODO: setup camera properties once info is obtained
            cameraSim = new PhotonCameraSim(camera, cameraProperties);
            // add camera and position it relative to robot
            visionSim.addCamera(cameraSim, VisionConstants.kCameraPos);
            cameraSim.enableDrawWireframe(true);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        /** 
         * Update the vision simulation. Should be called periodically. Requires the robot's
         * pose
         */
        private void updateSimulation(Pose2d robotPose) {
            visionSim.update(robotPose);
        }

        /**
         * Returns the estimated robot's position in simulation.
         * 
         * Basically taken from PhotonLib examples.
         */
        private Optional<EstimatedRobotPose> getEstimatedRobotPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var change : camera.getAllUnreadResults()) {
                visionEst = photonPoseEstimator.update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());

                visionEst.ifPresentOrElse(
                    est ->
                        visionSim.getDebugField()
                                 .getObject("VisionEstimation")
                                 .setPose(est.estimatedPose.toPose2d()),
                    () -> 
                        visionSim.getDebugField().getObject("VisionEstimation").setPoses()
                );
            }
            return visionEst;
        }

        /** Stolen from Photonlib examples. */
        private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if (estimatedPose.isEmpty()) {
                // No pose input. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;

            } else {
                // Pose present. Start running Heuristic
                var estStdDevs = kSingleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;

                // Precalculation - see how many tags we found, and calculate an average-distance metric
                for (var tgt : targets) {
                    var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isEmpty()) continue;
                    numTags++;
                    avgDist +=
                        tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }

                if (numTags == 0) {
                    // No tags visible. Default to single-tag std devs
                    curStdDevs = kSingleTagStdDevs;
                } else {
                    // One or more tags visible, run the full heuristic.
                    avgDist /= numTags;
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                    // Increase std devs based on (average) distance
                    if (numTags == 1 && avgDist > 4)
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                    curStdDevs = estStdDevs;
                }
            }
        }

        private  Matrix<N3, N1> getEstimationStdDevs() {
            return curStdDevs;
        }

        private Field2d getSimDebugField() {
            return visionSim.getDebugField();
        }
    }
}
