package frc.robot.vision;

import static frc.robot.vision.LimelightHelpers.*;

public class Vision {
    private Vision() {}

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
}
