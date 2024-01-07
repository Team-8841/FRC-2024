package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SimConstants;

public class SimSwerveModuleIO implements SwerveModuleIO {
    DCMotorSim steeringMotor = new DCMotorSim(SimConstants.steeringGearbox,
            SimConstants.steeringGearRatio, SimConstants.steeringInertia);
    DCMotorSim driveMotor = new DCMotorSim(SimConstants.driveGearbox, SimConstants.driveGearRatio,
            SimConstants.driveInertia);

    PIDController steeringPID = new PIDController(SimConstants.steeringKP, SimConstants.steeringKI,
            SimConstants.steeringKD);

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0, SimConstants.driveKV);
    PIDController drivePID = new PIDController(SimConstants.driveKP, SimConstants.driveKI,
            SimConstants.driveKD);

    private double totalDistanceDrove = 0, lastVelocity = 0;
    private long lastTimestamp;

    public SimSwerveModuleIO() {
        this.steeringPID.setTolerance(0.1);
        this.steeringPID.enableContinuousInput(0, 360);

        this.drivePID.setTolerance(0.1);
        this.lastTimestamp = Logger.getInstance().getTimestamp();
    }

    private double rad2WheelSpeed(double rad) {
        return rad * SimConstants.wheelDiameter / 2.0;
    }

    private double wheelSpeed2Rad(double wheelSpeed) {
        return wheelSpeed * 2.0 / SimConstants.wheelDiameter;
    }

    @Override
    public void periodic() {
        double steeringPower = this.steeringPID
                .calculate(Units.radiansToDegrees(this.steeringMotor.getAngularPositionRad()));

        if (!this.steeringPID.atSetpoint()) {
            this.steeringMotor.setInputVoltage(steeringPower);
        } else {
            this.steeringMotor.setInputVoltage(0);
        }

        double driveFeedForward = this.driveFeedForward.calculate(this.drivePID.getSetpoint());
        double drivePower = this.drivePID.calculate(this.driveMotor.getAngularVelocityRadPerSec())
                + driveFeedForward;

        if (!this.drivePID.atSetpoint()) {
            this.driveMotor.setInputVoltage(drivePower);
        } else {
            this.driveMotor.setInputVoltage(driveFeedForward);
        }

        long lastTimestamp = this.lastTimestamp;
        long currentTimestamp = Logger.getInstance().getTimestamp();
        this.lastTimestamp = currentTimestamp;
        double dt = (currentTimestamp - lastTimestamp) / 1000000.0;

        double currentVelocity = this.rad2WheelSpeed(this.driveMotor.getAngularVelocityRadPerSec());
        this.totalDistanceDrove += (Math.abs(currentVelocity) + Math.abs(this.lastVelocity)) / 2 * dt;
        this.lastVelocity = currentVelocity;

        this.steeringMotor.update(dt);
        this.driveMotor.update(dt);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        this.steeringPID.setSetpoint(desiredState.angle.getDegrees());
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        this.drivePID.setSetpoint(this.wheelSpeed2Rad(desiredState.speedMetersPerSecond));
    }

    @Override
    public SwerveModuleState getState() {
        double speedMetersPerSecond = this.rad2WheelSpeed(this.driveMotor.getAngularVelocityRadPerSec());
        Rotation2d angle = Rotation2d.fromRadians(this.steeringMotor.getAngularPositionRad());

        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                this.totalDistanceDrove,
                Rotation2d.fromRadians(this.steeringMotor.getAngularPositionRad()));
    }
}