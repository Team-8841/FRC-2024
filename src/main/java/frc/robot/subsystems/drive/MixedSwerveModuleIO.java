package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.swerve.MixedMotorConstants;
import frc.robot.constants.swerve.SwerveConstants;

public final class MixedSwerveModuleIO implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private TalonFX steeringMotor;
    private CANcoder steeringEncoder;
    private SparkMaxPIDController sparkMaxPID;
    private StatusSignal<Double> angleSignal, angleStatorSignal, angleSupplySignal;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(MixedMotorConstants.driveKS, MixedMotorConstants.driveKV, MixedMotorConstants.driveKA);
    private Rotation2d lastAngle = Rotation2d.fromDegrees(0);
    private PositionDutyCycle drivePosDutyCycle = new PositionDutyCycle(0).withSlot(0);
    private SwerveModuleConstants constants;


    /**
     * Creates a new container from each of the components.
     * 
     * @param driveMotor      The TalonFX controller controlling the drive motor.
     * @param steeringMotor   The SparkMax controller controlling the steering
     *                        motor.
     * @param steeringEncoder The CTRE CANCoder attached to the steering motor.
     * @param constants Module specific constants.
     */
    public MixedSwerveModuleIO(CANSparkMax driveMotor, TalonFX steeringMotor, CANcoder steeringEncoder,
            SwerveModuleConstants constants) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.steeringEncoder = steeringEncoder;
        this.constants = constants;

        this.configSteeringEncoder();
        this.configDriveMotor();
        this.configSteeringMotor();

        angleSignal = this.steeringEncoder.getAbsolutePosition();
        angleStatorSignal = this.steeringMotor.getStatorCurrent();
        angleSupplySignal = this.steeringMotor.getSupplyCurrent();
    }

    /**
     * Creates a new container from each of the components.
     * 
     * @param constants Module specific constants.
     */
    public MixedSwerveModuleIO(SwerveModuleConstants constants) {
        this(
                new CANSparkMax(constants.driveMotorID, MotorType.kBrushless),
                new TalonFX(constants.angleMotorID),
                new CANcoder(constants.cancoderID),
                constants);
    }

    private void configSteeringEncoder() {
        CANcoderConfigurator configurator = this.steeringEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = SwerveConstants.canCoderDir;
        magnetSensorConfigs.MagnetOffset = this.constants.angleOffset.getRotations();

        configurator.apply(magnetSensorConfigs);
    }

    private void configDriveMotor() {
        this.driveMotor.restoreFactoryDefaults();
        this.driveMotor.setInverted(SwerveConstants.driveMotorInvert == InvertedValue.Clockwise_Positive);
        this.driveMotor.setSmartCurrentLimit(SwerveConstants.drivePeakCurrentLimit,
                SwerveConstants.driveContinuousCurrentLimit);
        this.driveMotor.setIdleMode(MixedMotorConstants.driveNeutralMode);

        this.sparkMaxPID = this.driveMotor.getPIDController();
        this.sparkMaxPID.setP(MixedMotorConstants.driveKP);
        this.sparkMaxPID.setI(MixedMotorConstants.driveKI);
        this.sparkMaxPID.setD(MixedMotorConstants.driveKD);
        // Feedforward (kV) is supplied externally
        this.sparkMaxPID.setFF(0);
    }

    private void configSteeringMotor() {
        var configurator = this.steeringMotor.getConfigurator();
        configurator.apply(MixedMotorConstants.angleMotorConfigs);

        var feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = this.steeringEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.RotorToSensorRatio = SwerveConstants.angleGearRatio;
        configurator.apply(feedbackConfigs);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        // Closed loop
        double velocity = Conversions.metersToRots(desiredState.speedMetersPerSecond,
                SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
        double ff = this.driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        this.sparkMaxPID.setReference(velocity * 60, ControlType.kVelocity, 0, ff);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        var angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;
        this.lastAngle = angle;
        this.steeringMotor.setControl(this.drivePosDutyCycle.withPosition(angle.getRotations()));
    }

    @Override
    public SwerveModuleState getState() {
        double angle = this.angleSignal.refresh().getValue();

        return new SwerveModuleState(
                Conversions.rotsToMeters(this.driveMotor.getEncoder().getVelocity() / 60,
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public SwerveModulePosition getPosition() {
        double angle = this.angleSignal.refresh().getValue();

        return new SwerveModulePosition(
                Conversions.rotsToMeters(this.driveMotor.getEncoder().getPosition(),
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
        layout.addDouble("Angle", () -> this.getAngle().getDegrees());
        layout.addDouble("Speed", this::getSpeed);
        layout.addDouble("Position", () -> this.getPosition().distanceMeters);
        layout.addDouble("Angle Motor Stator Current", () -> this.angleStatorSignal.refresh().getValue());
        layout.addDouble("Angle Motor Supply Current", () -> this.angleSupplySignal.refresh().getValue());
        layout.addDouble("Drive Motor Output Current", () -> this.driveMotor.getOutputCurrent());
    }
}
