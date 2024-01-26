package frc.robot.sensors.vision.poseestimation;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.VisionState;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraConfig;
import frc.robot.sensors.vision.Camera;

public class LimelightEstimatorModuleIO implements EstimatorModuleIO {
    private NetworkTable table;
    private long lastChange, lastUpdate;
    private PoseEstimate lastPoseEstimate;
    private CameraConfig config;
    private String name;

    public LimelightEstimatorModuleIO(CameraConfig config) {
        this.config = config;
    }

    @Override
    public boolean updateInputs(EstimatorInputsAutoLogged inputs) {
        if (this.lastChange <= this.lastUpdate) {
            return false;
        }
        this.lastChange = RobotController.getFPGATime();

        if (this.lastPoseEstimate != null) {
            inputs.lastPoseTimestamp = this.lastChange;

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

        inputs.bestTgtArea = this.table.getEntry("ta").getDouble(-1);
        inputs.bestTgtId = (int) this.table.getEntry("tid").getInteger(-1);

        if (inputs.bestTgtId > 0) {
            var tagPose = VisionConstants.fieldLayout.getTagPose(inputs.bestTgtId).get();
            inputs.bestTgtDist = this.lastPoseEstimate.estimatedPose.getTranslation()
                    .getDistance(tagPose.getTranslation().toTranslation2d());
        }

        inputs.latency = this.table.getEntry("tl").getDouble(0) + this.table.getEntry("cl").getDouble(0);

        return true;
    }

    @Override
    public void setCamera(Camera camera) {
        this.table = NetworkTableInstance.getDefault().getTable(camera.name);
        this.table.getEntry("pipeline").setInteger(this.config.apriltagPipeline);
        this.table.getEntry("ledMode").setInteger(1);
        this.name = camera.name;
    }

    @Override
    public Optional<PoseEstimate> getPoseEstimation(Pose2d currentEstimatedPose) {
        NetworkTableEntry poseEntry;

        if (this.table.getEntry("getpipe").getInteger(-1) != this.config.apriltagPipeline
                || this.table.getEntry("tv").getInteger(0) != 1) {
            return Optional.empty();
        }

        if (DriverStation.getAlliance().orElse(Constants.defaultAlliance) == DriverStation.Alliance.Blue) {
            poseEntry = this.table.getEntry("botpose_wpiblue");
        } else {
            poseEntry = this.table.getEntry("botpose_wpired");
        }

        if (poseEntry.getLastChange() <= this.lastChange) {
            return Optional.empty();
        }
        this.lastChange = poseEntry.getLastChange();

        double[] poseComponents = poseEntry.getDoubleArray(new double[6]);
        var estimatedTrans = new Translation3d(poseComponents[0], poseComponents[1], poseComponents[2]);
        var estimatedRot = new Rotation3d(poseComponents[3], poseComponents[4], poseComponents[5]);

        // TODO: Check if we need to further transform pose to get from limelight field
        // space to wpilib field space
        var estimatedPose = new Pose3d(estimatedTrans, estimatedRot).toPose2d();

        var prefix = String.format("/Vision/%s/", this.name);
        Logger.recordOutput(prefix + "rawEstPose", estimatedPose);

        // Filters estimates greater than 1 meter away from the current estimated
        // position
        if (estimatedPose.getTranslation().getDistance(currentEstimatedPose.getTranslation()) > 1) {
            return Optional.empty();
        }

        Matrix<N3, N1> estimatedStdDevs;
        long tid = this.table.getEntry("tid").getInteger(-1);
        if (tid < 0) {
            estimatedStdDevs = VisionConstants.singleTagStdDevs;
        } else {
            var tags = new ArrayList<Integer>();
            tags.add(Integer.valueOf((int) tid));
            estimatedStdDevs = VisionState.getEstimationStdDevs(tags, currentEstimatedPose);
        }

        Logger.recordOutput(prefix + "lastEstPose", estimatedPose);
        Logger.recordOutput(prefix + "lastStdDevs", estimatedStdDevs.getData());

        var estimate = new PoseEstimate(estimatedPose, estimatedStdDevs);
        this.lastPoseEstimate = estimate;

        return Optional.of(estimate);
    }
}
