package frc.robot.sensors.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.VisionConstants;

public class LimelightEstimatorModuleIO implements EstimatorModuleIO {
    NetworkTable table;
    long lastChange;

    public LimelightEstimatorModuleIO() {
    }

    @Override
    public boolean updateInputs(EstimatorInputsAutoLogged inputs) {
        return false;
    }

    @Override
    public void setCamera(EstimatorCamera camera) {
        this.table = NetworkTableInstance.getDefault().getTable(camera.name);
    }

    @Override
    public Optional<PoseEstimate> getPoseEstimation(Pose2d currentEstimatedPose) {
        NetworkTableEntry poseEntry = this.table.getEntry("botpose");

        if (poseEntry.getLastChange() <= this.lastChange) {
            return Optional.empty();
        }
        this.lastChange = poseEntry.getLastChange();

        double[] poseComponents = poseEntry.getDoubleArray(new double[6]);
        Translation3d estimatedTrans = new Translation3d(poseComponents[0], poseComponents[1], poseComponents[2]);
        Rotation3d estimatedRot = new Rotation3d(poseComponents[3], poseComponents[4], poseComponents[5]);
        Pose3d estimatedPose = new Pose3d(estimatedTrans, estimatedRot);

        return Optional.of(
                new PoseEstimate(estimatedPose.toPose2d(), VisionConstants.singleTagStdDevs));
    }
}
