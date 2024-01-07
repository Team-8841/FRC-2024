package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.sensors.imu.SimIMU;

public class SimManager {
    private static SimManager instance;

    private SimIMU imu;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedSupplier;

    private SimManager() {
        if (!RobotBase.isSimulation()) {
            throw new Error("Cannot open SimManager outside of a simulation.");
        }
    }

    public static SimManager getInstance() {
        if (instance == null) {
            SimManager.instance = new SimManager();
        }

        return SimManager.instance;
    }

    public void registerIMU(SimIMU imu) {
        this.imu = imu;
    }

    public void registerDriveTrain(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedSupplier = speedSupplier;

        this.imu.registerOrientationSuppliers(speedSupplier);
    }

    public void periodic() {
    }
}
