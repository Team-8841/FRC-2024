package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double setSpeedMetersPerSecond, actualSpeedMetersPerSecond;
        public double setAngle, actualAngle;
        public double distance;
    }

    /**
     * Called on every call to the parent subsystem periodic().
     */
    public default void periodic() {
    }

    /**
     * Resets the module to a known state.
     */
    public default void reset() {
    }

    public default void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
    }

    public default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        SwerveModuleState state = this.getState();
        inputs.actualSpeedMetersPerSecond = state.speedMetersPerSecond;
        inputs.actualAngle = state.angle.getDegrees();
        inputs.distance = this.getPosition().distanceMeters;
    }

    /**
     * Set the desired state of the swerve module.
     * 
     * @param desiredState The desired state.
     */
    public default void setDesiredState(SwerveModuleState desiredState) {
        this.setAngle(desiredState);
        this.setSpeed(desiredState);
    }

    /**
     * Resets the steering PID constants of the swerve module. The implementor is
     * not required to implement this.
     */
    public default void resetSteeringPID() {
    }

    /**
     * Sets the steering PID constants of the swerve module. The implementor is not
     * required to implement this.
     * 
     * @param kS
     * @param kV
     * @param kA
     * @param kP
     * @param kI
     * @param kD
     */
    public default void setSteeringPID(double kS, double kV, double kA, double kP, double kI, double kD) {
    }

    /**
     * Resets the drive PID constants of the swerve module. The implementor is not
     * required to implement this.
     */
    public default void resetDrivePID() {
    }

    /**
     * Sets the drive PID constants of the swerve module. The implementor is not
     * required to implement this.
     * 
     * @param kS
     * @param kV
     * @param kA
     * @param kP
     * @param kI
     * @param kD
     */
    public default void setDrivePID(double kS, double kV, double kA, double kP, double kI, double kD) {
    }

    /**
     * Set the angle of the swerve module.
     * 
     * @param desiredState State which contains the desired angle.
     */
    public void setAngle(SwerveModuleState desiredState);

    /**
     * Set the speed of the swerve module.
     * 
     * @param desiredState State which contains the desired speed.
     */
    public void setSpeed(SwerveModuleState desiredState);

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The current state.
     */
    public SwerveModuleState getState();

    /**
     * Gets the current position of the swerve module.
     * 
     * @return The current position.
     */
    public SwerveModulePosition getPosition();

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The current state.
     */
    public default Rotation2d getAngle() {
        return this.getState().angle;
    }

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The current state.
     */
    public default double getSpeed() {
        return this.getState().speedMetersPerSecond;
    }

    /**
     * Gets the total supplied current to the swerve module. Not required to be
     * implemented.
     * 
     * @return The total supplied current (amps).
     */
    public default double getCurrent() {
        return 0;
    }
}
