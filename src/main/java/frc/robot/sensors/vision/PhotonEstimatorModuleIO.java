package frc.robot.sensors.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.PhotonConstants;
import frc.robot.constants.VisionConstants;

public class PhotonEstimatorModuleIO implements EstimatorModuleIO {
    private final PhotonConstants.EstimatorConfig config;
    private double lastResult;
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private PoseEstimate lastPoseEstimate;

    public PhotonEstimatorModuleIO(PhotonConstants.EstimatorConfig config) {
        this.config = config;
    }

    /**
     * The standard deviations of the estimated pose the estimator for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    private Matrix<N3, N1> getEstimationStdDevs(List<PhotonTrackedTarget> targets, Pose2d estimatedPose) {
        Matrix<N3, N1> estStdDevs = VisionConstants.singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (PhotonTrackedTarget tgt : targets) {
            Optional<Pose3d> tagPose = VisionConstants.fieldLayout.getTagPose(tgt.getFiducialId());
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

    @Override
    public boolean updateInputs(EstimatorInputsAutoLogged inputs) {
        PhotonPipelineResult latestResult = this.camera.getLatestResult();

        if (latestResult.getTimestampSeconds() <= inputs.timestamp) {
            return false;
        }

        inputs.timestamp = latestResult.getTimestampSeconds();
        inputs.latency = latestResult.getLatencyMillis();
        inputs.isConnected = this.camera.isConnected();

        if (this.lastPoseEstimate != null) {
            inputs.lastPoseTimestamp = this.lastPoseEstimate.timestamp;

            inputs.estPose = new double[] {
                    this.lastPoseEstimate.estimatedPose.getX(),
                    this.lastPoseEstimate.estimatedPose.getY(),
                    this.lastPoseEstimate.estimatedPose.getRotation().getRadians(),
            };
            inputs.stdDevs = new double[] {
                    this.lastPoseEstimate.stdDevs.get(0, 0),
                    this.lastPoseEstimate.stdDevs.get(1, 0),
                    this.lastPoseEstimate.stdDevs.get(2, 0)
            };
        }

        if (latestResult.hasTargets()) {
            inputs.targetCount = latestResult.getTargets().size();

            PhotonTrackedTarget bestTgt = latestResult.getBestTarget();

            if (this.lastPoseEstimate != null) {

                Pose2d tagPose = VisionConstants.fieldLayout.getTagPose(bestTgt.getFiducialId()).get().toPose2d();
                inputs.bestTgtDist = this.lastPoseEstimate.estimatedPose.getTranslation()
                        .getDistance(tagPose.getTranslation());
            }

            inputs.bestTgtArea = bestTgt.getArea();
            inputs.bestTgtID = bestTgt.getArea();
            inputs.bestTgtAmbiguity = bestTgt.getPoseAmbiguity();

            List<TargetCorner> corners = bestTgt.getDetectedCorners();

            inputs.estTgtCornersX = new double[corners.size()];
            inputs.estTgtCornersY = new double[corners.size()];

            int i = 0;
            for (TargetCorner corner : corners) {
                inputs.estTgtCornersX[i] = corner.x;
                inputs.estTgtCornersY[i++] = corner.y;
            }
        } else {
            inputs.targetCount = 0;
        }

        return true;
    }

    @Override
    public void setCamera(EstimatorCamera camera) {
        System.out.println("Connecting to camera: " + camera);

        this.camera = new PhotonCamera(camera.name);

        this.camera.setPipelineIndex(0);
        this.camera.setDriverMode(false);
        this.camera.setLED(VisionLEDMode.kOff);

        this.estimator = new PhotonPoseEstimator(VisionConstants.fieldLayout, this.config.strategy,
                this.camera, this.config.robotToCamera);
    }

    @Override
    public Optional<PoseEstimate> getPoseEstimation(Pose2d currentEstimatedPose) {
        if (!this.camera.isConnected()) {
            return Optional.empty();
        }

        PhotonPipelineResult pipelineResult = this.camera.getLatestResult();

        // If the this is an old pipeline result return none
        if (pipelineResult.getTimestampSeconds() <= this.lastResult || !pipelineResult.hasTargets()) {
            return Optional.empty();
        }
        this.lastResult = pipelineResult.getTimestampSeconds();

        // Filters targets based on minimum area
        List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
        for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getArea() < VisionConstants.minimumTagArea) {
                targets.remove(i--);
            }
        }

        // If no targets are left return none
        if (targets.size() == 0) {
            return Optional.empty();
        }

        // Calculates the standard deviation of the estimated pose
        Matrix<N3, N1> stdDevs = this.getEstimationStdDevs(targets, currentEstimatedPose);

        // Calculates the estimated pose
        PhotonPipelineResult filteredPipelineResult = new PhotonPipelineResult(pipelineResult.getLatencyMillis(),
                targets);
        filteredPipelineResult.setTimestampSeconds(pipelineResult.getTimestampSeconds());
        Optional<EstimatedRobotPose> visionPoseOpt = this.estimator.update(filteredPipelineResult);

        if (visionPoseOpt.isEmpty()) {
            return Optional.empty();
        }

        Pose2d estimatedPose = visionPoseOpt.get().estimatedPose.toPose2d();
        PoseEstimate estimate = new PoseEstimate(estimatedPose, stdDevs);
        this.lastPoseEstimate = estimate;

        return Optional.of(estimate);
    }
}
