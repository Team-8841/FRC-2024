package frc.robot.sensors.imu;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Interface used by all IMU's. All implementing classes HAVE TO output
 * orientations as CCW+ (Counter-clockwise positive).
 */
public abstract class IMU extends SubsystemBase {
    private IMUInputsAutoLogged inputs = new IMUInputsAutoLogged();
    
    @AutoLog
    public static class IMUInputs {
        double heading, pitch, yaw, roll;
        double qX, qY, qZ, qW;
        boolean isInitialized, isSensorPresent;
    }

    @Override
    public void periodic() {
        this.updateInputs(this.inputs);
        Logger.getInstance().processInputs("/IMU/Inputs", this.inputs);
    }

    protected void updateInputs(IMUInputsAutoLogged inputs) {
        inputs.heading = this.getHeading().getDegrees();

        Rotation3d orientation = this.getOrientation();
        inputs.pitch = orientation.getY();
        inputs.yaw = orientation.getZ();
        inputs.roll = orientation.getX();

        Quaternion quaternion = orientation.getQuaternion();
        inputs.qX = quaternion.getX();
        inputs.qY = quaternion.getY();
        inputs.qZ = quaternion.getZ();
        inputs.qW = quaternion.getW();

        inputs.isInitialized = this.isInitialized();
        inputs.isSensorPresent = this.isSensorPresent();
    }

    public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
        layout.addDouble("Heading", () -> this.getHeading().getDegrees()).withWidget(BuiltInWidgets.kGyro);
        layout.addDouble("Yaw", () -> this.getYaw().getDegrees());
        layout.addDouble("Pitch", () -> this.getPitch().getDegrees());
        layout.addDouble("Roll", () -> this.getRoll().getDegrees());
        layout.addBoolean("Is Initialized", this::isInitialized);
        layout.addBoolean("Is Sensor Present", this::isSensorPresent);
    }

    /**
     * Get's the IMU's current orientation.
     * 
     * @return The current 3d orientation (pitch, roll, and yaw) of the IMU as a
     *         Rotation3d.
     */
    public abstract Rotation3d getOrientation();

    /**
     * Gets the IMU's current heading angle.
     * 
     * @return The current heading angle as a Rotation2d.
     */
    public abstract Rotation2d getHeading();

    /**
     * Gets the IMU's current pitch angle.
     * 
     * @return The current pitch angle as a Rotation2d
     */
    public Rotation2d getPitch() {
        return Rotation2d.fromRadians(this.getOrientation().getY());
    }

    /**
     * Gets the IMU's current yaw angle.
     * 
     * @return The current yaw angle as a Rotation2d.
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromRadians(this.getOrientation().getZ());
    }

    /**
     * Gets the IMU's current roll angle.
     * 
     * @return The current roll angle as a Rotation2d.
     */
    public Rotation2d getRoll() {
        return Rotation2d.fromRadians(this.getOrientation().getX());
    }

    /**
     * Get's if the IMU has been initialized.
     * 
     * @return If the IMU has been initialized.
     */
    public abstract boolean isInitialized();

    /**
     * Get's if the IMU is present.
     * 
     * @return If the IMU is present.
     */
    public abstract boolean isSensorPresent();
}