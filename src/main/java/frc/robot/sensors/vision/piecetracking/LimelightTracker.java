package frc.robot.sensors.vision.piecetracking;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.VisionConstants.CameraConfig;
import frc.robot.sensors.vision.Camera;

public class LimelightTracker extends TrackerCommand {
    private NetworkTable table;
    private CameraConfig config;

    public LimelightTracker(Camera camera, CameraConfig config) {
        super(camera);
        this.config = config;
    }

    @Override
    protected void setCamera(Camera camera) {
        this.table = NetworkTableInstance.getDefault().getTable(camera.name);
        this.table.getEntry("pipeline").setInteger(this.config.trackerPipeline);
        this.table.getEntry("ledMode").setInteger(1);
    }

    @Override
    protected boolean updateInputs(TrackerModuleInputsAutoLogged inputs) {
        inputs.latency = this.table.getEntry("cl").getDouble(0) + this.table.getEntry("tl").getDouble(0);
        inputs.targetX = this.table.getEntry("tx").getDouble(0);
        inputs.targetY = this.table.getEntry("ty").getDouble(0);
        inputs.targetArea = this.table.getEntry("targetArea").getDouble(0);
        inputs.targetPresent = this.table.getEntry("tv").getInteger(0) == 1;
        inputs.cameraPresent = this.table.getEntry("tv").exists();

        return true;
    }

    @Override
    public Optional<Rotation2d> getTargetAngle() {
        if (this.table.getEntry("getpipe").getInteger(-1) != this.config.trackerPipeline
                || this.table.getEntry("tv").getInteger(0) != 1) {
            return Optional.empty();
        }

        var rotEntry = this.table.getEntry("tx");
        var rot = Rotation2d.fromDegrees(rotEntry.getDouble(0));
        return rotEntry.exists() ? Optional.of(rot) : Optional.empty();
    }

}
