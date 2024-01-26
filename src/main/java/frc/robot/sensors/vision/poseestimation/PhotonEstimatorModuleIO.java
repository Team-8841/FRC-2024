package frc.robot.sensors.vision.poseestimation;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.VisionState;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.sensors.vision.Camera;

public class PhotonEstimatorModuleIO implements EstimatorModuleIO {
    private final VisionConstants.CameraConfig config;
    private double lastResult;
    private PhotonCamera camera;
    private PoseEstimate lastPoseEstimate;
    private Transform3d cameraToRobot;

    public PhotonEstimatorModuleIO(VisionConstants.CameraConfig config) {
        this.config = config;
        this.cameraToRobot = this.config.robotToCamera.inverse();
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
            inputs.bestTgtId = bestTgt.getFiducialId();
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
    public void setCamera(Camera camera) {
        System.out.println("Connecting to camera: " + camera.name);

        this.camera = new PhotonCamera(camera.name);

        this.camera.setPipelineIndex(this.config.apriltagPipeline);
        this.camera.setDriverMode(false);
        this.camera.setLED(VisionLEDMode.kOff);
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
        var stdDevs = VisionState.getEstimationStdDevs(multiTagResult.fiducialIDsUsed, currentEstimatedPose);

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

        Logger.recordOutput(prefix + "lastEstPose", estimatedPose);
        Logger.recordOutput(prefix + "lastStdDevs", stdDevs.getData());

        var estimate = new PoseEstimate(estimatedPose, stdDevs);
        this.lastPoseEstimate = estimate;

        return Optional.of(estimate);
    }
}
