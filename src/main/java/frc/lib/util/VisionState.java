package frc.lib.util;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;

public class VisionState {
    /**
     * The standard deviations of the estimated pose the estimator for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public static Matrix<N3, N1> getEstimationStdDevs(List<Integer> targetIds, Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targetIds) {
            Optional<Pose3d> tagPose = VisionConstants.fieldLayout.getTagPose(tgt);
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) {
            return estStdDevs;
        }
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = VisionConstants.multiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        // if (numTags == 1 && avgDist > 4) {
        // estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
        // Double.MAX_VALUE);
        // } else {
        // estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        // }

        return estStdDevs;
    }

}
