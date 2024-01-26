package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final double minimumTagArea = 0.2;

    public static final double minimumTrackerArea = 0.2;

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(1, 1, 2);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.7);

    public static final class CameraConfig {
        public final Transform3d robotToCamera;
        public final int apriltagPipeline, trackerPipeline;
        public final Rotation2d fov;

        public CameraConfig(int pipelineIndex, int trackerPipeline, Transform3d robotToCamera, Rotation2d fov) {
            this.apriltagPipeline = pipelineIndex;
            this.trackerPipeline = trackerPipeline;
            this.robotToCamera = robotToCamera;
            this.fov = fov;
        }
    }

    public static final CameraConfig frontCameraEstimator = new CameraConfig(0, -1, new Transform3d(), Rotation2d.fromDegrees(29.8));
}
