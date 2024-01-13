package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public final class PhotonConstants {
  public static final class EstimatorConfig {
    public final int pipelineIndex;
    public final PoseStrategy strategy;
    public final Transform3d robotToCamera;

    public EstimatorConfig(int pipelineIndex, PoseStrategy strategy, Transform3d robotToCamera) {
      this.pipelineIndex = pipelineIndex;
      this.strategy = strategy;
      this.robotToCamera = robotToCamera;
    }
  }

  public static final EstimatorConfig frontCameraEstimator =
      new EstimatorConfig(0, PoseStrategy.LOWEST_AMBIGUITY, new Transform3d());
}
