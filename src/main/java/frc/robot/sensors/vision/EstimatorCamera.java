package frc.robot.sensors.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class EstimatorCamera extends Camera {
    public EstimatorCamera(String name, EstimatorModuleIO moduleIO,
            SwerveDrivePoseEstimator poseEstimator) {
        super(name);
        this.setDefaultCommand(new PoseEstimatorCommand(this, moduleIO, poseEstimator));
    }
}
