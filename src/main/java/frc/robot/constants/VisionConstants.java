package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final double minimumTagArea = 0.2;

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(1, 1, 2);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.7);

    public static final class EstimatorConfig {
        public final int pipelineIndex;
        public final Transform3d robotToCamera;

        public EstimatorConfig(int pipelineIndex, Transform3d robotToCamera) {
            this.pipelineIndex = pipelineIndex;
            this.robotToCamera = robotToCamera;
        }
    }

    public static final EstimatorConfig frontCameraEstimator = new EstimatorConfig(0, new Transform3d());
}
