package frc.robot.sensors.vision.piecetracking;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import frc.robot.constants.VisionConstants.CameraConfig;
import frc.robot.sensors.vision.Camera;

public class PhotonTracker extends TrackerCommand {
    private PhotonCamera camera;
    private CameraConfig config;

    public PhotonTracker(Camera camera, CameraConfig config) {
        super(camera);
        this.config = config;
    }

    @Override
    public void setCamera(Camera camera) {
        System.out.println("Connecting to camera: " + camera.name);

        this.camera = new PhotonCamera(camera.name);

        this.camera.setPipelineIndex(this.config.trackerPipeline);
        this.camera.setDriverMode(false);
        this.camera.setLED(VisionLEDMode.kOff);
    }

    @Override
    public 
}
