package frc.robot.sensors.imu;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Pigeon2IO extends IMU {
    Pigeon2 pigeon;

    public Pigeon2IO(int canId) {
        this.pigeon = new Pigeon2IO(canId);
    }

    @Override
    public Rotation3d getOrientation() {
        return this.pigeon.getRotation3d();
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(this.pigeon.getAngle());
    }

    @Override
    public boolean isInitialized() {
        return true;
    }

    @Override
    public boolean isSensorPresent() {
        return true;
    }
}
