package frc.robot.sensors.vision.piecetracking;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.vision.Camera;

public abstract class TrackerCommand extends Command {
    private TrackerModuleInputsAutoLogged inputs;
    private Camera camera;

    @AutoLog
    public static class TrackerModuleInputs {
        double latency, targetArea, targetX, targetY;
        boolean cameraPresent, targetPresent;
    }

    protected TrackerCommand(Camera camera) {
        this.addRequirements(camera);
        this.camera = camera;
        this.setCamera(camera);
    }

    protected abstract void setCamera(Camera camera);

    protected abstract boolean updateInputs(TrackerModuleInputsAutoLogged inputs);

    public abstract Optional<Rotation2d> getTargetAngle();

    @Override
    public void execute() {
        if (this.updateInputs(inputs)) {
            Logger.processInputs(String.format("/Trackers/%s/", this.camera.name), inputs);
        }
    }
}