package frc.robot.sensors.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;

public class PhotonEstimatorModuleIO implements EstimatorModuleIO {
    private final VisionConstants.EstimatorConfig config;
    private double lastResult;
    private PhotonCamera camera;
    private PoseEstimate lastPoseEstimate;
    private Transform3d cameraToRobot;

    public PhotonEstimatorModuleIO(VisionConstants.EstimatorConfig config) {
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
    private Matrix<N3, N1> getEstimationStdDevs(List<Integer> targetIds, Pose2d estimatedPose) {
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

    @Override
    public boolean updateInputs(EstimatorInputsAutoLogged inputs) {
        var latestPipelineResult = this.camera.getLatestResult();

        if (latestPipelineResult.getTimestampSeconds() <= inputs.timestamp) {
            return false;
        }

        inputs.timestamp = latestPipelineResult.getTimestampSeconds();
        inputs.latency = latestPipelineResult.getLatencyMillis();
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

        if (latestPipelineResult.hasTargets()) {
            inputs.targetCount = latestPipelineResult.getTargets().size();

            var bestTgt = latestPipelineResult.getBestTarget();

            if (this.lastPoseEstimate != null) {

                Pose2d tagPose = VisionConstants.fieldLayout.getTagPose(bestTgt.getFiducialId()).get().toPose2d();
                inputs.bestTgtDist = this.lastPoseEstimate.estimatedPose.getTranslation()
                        .getDistance(tagPose.getTranslation());
            }

            inputs.bestTgtArea = bestTgt.getArea();
            inputs.bestTgtID = bestTgt.getArea();
            inputs.bestTgtAmbiguity = bestTgt.getPoseAmbiguity();

            var corners = bestTgt.getDetectedCorners();

            inputs.estTgtCornersX = new double[corners.size()];
            inputs.estTgtCornersY = new double[corners.size()];

            int i = 0;
            for (var corner : corners) {
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

        this.camera.setPipelineIndex(this.config.pipelineIndex);
        this.camera.setDriverMode(false);
        this.camera.setLED(VisionLEDMode.kOff);

        this.cameraToRobot = this.config.robotToCamera.inverse();
    }

    @Override
    public Optional<PoseEstimate> getPoseEstimation(Pose2d currentEstimatedPose) {
        if (!this.camera.isConnected()) {
            return Optional.empty();
        }

        var pipelineResult = this.camera.getLatestResult();
        var multiTagResult = pipelineResult.getMultiTagResult();

        // If the this is an old or empty pipeline result return none
        if (pipelineResult.getTimestampSeconds() <= this.lastResult || !pipelineResult.hasTargets()
                || !multiTagResult.estimatedPose.isPresent) {
            return Optional.empty();
        }
        this.lastResult = pipelineResult.getTimestampSeconds();

        // Calculates the standard deviation of the estimated pose
        var stdDevs = this.getEstimationStdDevs(multiTagResult.fiducialIDsUsed, currentEstimatedPose);

        var bestPoseTransform = multiTagResult.estimatedPose.best.plus(this.cameraToRobot);

        var prefix = String.format("/Vision/%s/", this.camera.getName());
        Logger.recordOutput(prefix + "rawEstTransform", multiTagResult.estimatedPose.best);

        var estimatedPose = new Pose3d(bestPoseTransform.getTranslation(), bestPoseTransform.getRotation()).toPose2d();
        Logger.recordOutput(prefix + "rawEstPose", estimatedPose);

        var alliance = DriverStation.getAlliance().orElse(Constants.defaultAlliance);
        if (alliance == DriverStation.Alliance.Red) {
            estimatedPose = new Pose2d(
                    VisionConstants.fieldLayout.getFieldWidth() - estimatedPose.getX(),
                    VisionConstants.fieldLayout.getFieldLength() - estimatedPose.getY(),
                    Rotation2d.fromDegrees(180).plus(estimatedPose.getRotation()));
        }

        // Filters estimates greater than 1 meter away from the current estimated
        // position
        if (estimatedPose.getTranslation().getDistance(currentEstimatedPose.getTranslation()) > 1) {
            return Optional.empty();
        }

        var estimate = new PoseEstimate(estimatedPose, stdDevs);
        this.lastPoseEstimate = estimate;

        return Optional.of(estimate);
    }
}
