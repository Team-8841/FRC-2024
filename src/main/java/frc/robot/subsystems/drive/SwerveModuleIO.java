package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public LoggableInputs extraInfo;
        
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
}