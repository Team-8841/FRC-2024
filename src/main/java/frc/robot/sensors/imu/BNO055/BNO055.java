package frc.robot.sensors.imu.BNO055;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.sensors.imu.IMU;

/**
 * IMU container for the BNO055.
 */
public class BNO055 extends IMU {
    private BNO055IO internalBNO055;

    public BNO055() {
        this.internalBNO055 = BNO055IO.getInstance(BNO055IO.opmode_t.OPERATION_MODE_IMUPLUS,
                BNO055IO.vector_type_t.VECTOR_EULER);
    }

    @Override
    public Rotation3d getOrientation() {
        double[] vector = this.internalBNO055.getVector();

        return new Rotation3d(vector[1], vector[2], vector[0]);
    }

    @Override
    public Rotation2d getHeading() {
        return this.getYaw();
    }

    @Override
    public boolean isInitialized() {
        return this.internalBNO055.isInitialized();
    }

    @Override
    public boolean isSensorPresent() {
        return this.internalBNO055.isSensorPresent();
    }
}
