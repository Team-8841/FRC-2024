package frc.robot.sensors.imu;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class DummyIMU extends IMU {
    IMUInputsAutoLogged inputs = new IMUInputsAutoLogged();

    @Override
    public void updateInputs(IMUInputsAutoLogged inputsAutoLogged) {
        this.inputs = inputsAutoLogged;
    }

    @Override
    public Rotation3d getOrientation() {
        return new Rotation3d(
                new Quaternion(
                        this.inputs.qW,
                        this.inputs.qX,
                        this.inputs.qY,
                        this.inputs.qZ));
    }

    @Override 
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(this.inputs.heading);
    }

    @Override
    public boolean isInitialized() {
        return this.inputs.isInitialized;
    }

    @Override
    public boolean isSensorPresent() {
        return this.inputs.isSensorPresent;
    }
}
