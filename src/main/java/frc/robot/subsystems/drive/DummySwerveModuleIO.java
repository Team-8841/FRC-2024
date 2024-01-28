package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DummySwerveModuleIO implements SwerveModuleIO {
    SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        this.inputs = inputs;
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Do nothing
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        // Do nothing
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this.inputs.actualSpeedMetersPerSecond,
                Rotation2d.fromDegrees(this.inputs.actualAngle));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                this.inputs.distance,
                Rotation2d.fromDegrees(this.inputs.actualAngle));
    }
}
