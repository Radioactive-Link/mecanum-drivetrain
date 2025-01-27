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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.VisionConstants;

/** 
 * Used for fetching data from Vision APIs.
 * 
 * Designed with Limelight in mind, but could be refactored for use with Photonlib since that is
 * already used for simulation.
 */
public class Vision {
    public Vision() {
        var cameraPos = VisionConstants.kCameraPos;
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.kCameraName,
            cameraPos.getX(),
            cameraPos.getY(),
            cameraPos.getZ(),
            cameraPos.getRotation().getX(),
            cameraPos.getRotation().getY(),
            cameraPos.getRotation().getZ()
        );
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
    public static class VisionSim {
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

        public VisionSim() {
            // add all simulated april tag targets
            visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));
            // TODO: setup camera properties once info is obtained
            cameraSim = new PhotonCameraSim(camera, cameraProperties);
            // add camera and position it relative to robot
            visionSim.addCamera(cameraSim, VisionConstants.kCameraPos);
            cameraSim.enableDrawWireframe(true);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        /** Rotate the Simulated Camera. Useful if camera is mounted on a moving system. */
        public void rotateCamera(Rotation3d rotation) {
            visionSim.adjustCamera(
                cameraSim,
                new Transform3d(
                    VisionConstants.kCameraPos.getTranslation().rotateBy(rotation),
                    VisionConstants.kCameraPos.getRotation().rotateBy(rotation))
            );
        }

        /** 
         * Update the vision simulation. Should be called periodically. Requires the robot's
         * pose
         */
        public void updateSimulation(Pose2d robotPose) {
            visionSim.update(robotPose);
        }

        /**
         * Returns the estimated robot's position in simulation.
         * 
         * Basically taken from PhotonLib examples.
         */
        public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
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

        public Matrix<N3, N1> getEstimationStdDevs() {
            return curStdDevs;
        }

        public Field2d getSimDebugField() {
            return visionSim.getDebugField();
        }
    }
}
