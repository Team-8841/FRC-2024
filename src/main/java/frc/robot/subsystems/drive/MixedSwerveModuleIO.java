package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.swerve.BaseRobotConstants;
import frc.robot.constants.swerve.CompRobotConstants;
import frc.robot.constants.swerve.SwerveConstants;

// TODO: Make this use motorsafety

public class MixedSwerveModuleIO implements SwerveModuleIO {
    private boolean driveMotionMagic;

    private TalonFX driveMotor;

    private CANSparkMax angleMotor;
    private PIDController anglePID = new PIDController(BaseRobotConstants.angleKP, BaseRobotConstants.angleKI,
            BaseRobotConstants.angleKD);
    private CANcoder steeringEncoder;

    private Rotation2d lastAngle = new Rotation2d();

    private MotionMagicVelocityVoltage driveMagicVelVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);
    private VelocityVoltage driveVelVoltage = new VelocityVoltage(0).withSlot(0);

    private StatusSignal<Double> driveSupCur, driveStaCur, driveVel, drivePos, angleAngle;

    private SwerveModuleConstants constants;

    /**
     * Creates a new container from each of the components.
     * 
     * @param driveMotor      The TalonFX controller controlling the drive motor.
     * @param steeringMotor   The SparkMax controller controlling the steering
     *                        motor.
     * @param steeringEncoder The CTRE CANCoder attached to the steering motor.
     * @param constants       Module specific constants.
     */
    public MixedSwerveModuleIO(TalonFX driveMotor, CANSparkMax angleMotor, CANcoder steeringEncoder,
            SwerveModuleConstants constants, boolean driveMotionMagic) {
        this.driveMotionMagic = driveMotionMagic;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.steeringEncoder = steeringEncoder;
        this.constants = constants;

        this.configSteeringEncoder();
        this.configDriveMotor();
        this.configSteeringMotor();

        this.driveVel = this.driveMotor.getVelocity();
        this.drivePos = this.driveMotor.getPosition();
        this.driveSupCur = this.driveMotor.getSupplyCurrent();
        this.driveStaCur = this.driveMotor.getStatorCurrent();
        this.angleAngle = this.steeringEncoder.getAbsolutePosition();

        this.anglePID.setTolerance(Units.degreesToRotations(1));
        this.anglePID.enableContinuousInput(0, 1);
    }

    /**
     * Creates a new container from each of the component's CAN ID's.
     * 
     * @param constants Module specific constants.
     */
    public MixedSwerveModuleIO(SwerveModuleConstants constants, boolean driveMotionMagic) {
        this(
                new TalonFX(constants.driveMotorID),
                new CANSparkMax(constants.angleMotorID, MotorType.kBrushless),
                new CANcoder(constants.cancoderID),
                constants,
                driveMotionMagic);
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
        this.driveMotor.getConfigurator().apply(CompRobotConstants.driveMotorConfigs);
    }

    private void configSteeringMotor() {
        this.angleMotor.restoreFactoryDefaults();
        this.angleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
    }

    @Override
    public void setDrivePID(double kS, double kV, double kA, double kP, double kI, double kD) {
        var configurator = this.driveMotor.getConfigurator();
        var pidConfigs = new Slot0Configs();
        pidConfigs.kS = kS;
        pidConfigs.kV = kV;
        pidConfigs.kA = kA;
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        configurator.apply(pidConfigs);
    }

    @Override
    public void setSteeringPID(double kS, double kV, double kA, double kP, double kI, double kD) {
        this.anglePID.setP(kP);
        this.anglePID.setI(kI);
        this.anglePID.setD(kD);
        // Feedforward consts are ignored
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        // Closed loop
        double velocity = Conversions.metersToRots(desiredState.speedMetersPerSecond,
                SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
        if (this.driveMotionMagic) {
            this.driveMotor.setControl(this.driveMagicVelVoltage.withVelocity(velocity));
        } else {
            this.driveMotor.setControl(this.driveVelVoltage.withVelocity(velocity));
        }
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;
        this.lastAngle = angle;
    }

    @Override
    public SwerveModuleState getState() {
        double angle = this.angleAngle.refresh().getValue();
        return new SwerveModuleState(
                Conversions.rotsToMeters(this.driveVel.refresh().getValue(),
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public SwerveModulePosition getPosition() {
        double angle = this.angleAngle.refresh().getValue();
        return new SwerveModulePosition(
                Conversions.rotsToMeters(this.drivePos.refresh().getValue(),
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public double getCurrent() {
        return this.driveSupCur.getValue() + this.angleMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // Angle PID loop
        {
            double curAngle = this.angleAngle.refresh().getValueAsDouble();
            double setAngle = this.lastAngle.getRotations();
            double effort = this.anglePID.calculate(curAngle, setAngle);
            this.angleMotor.set(effort);
        }
        
        // Logging
        {
            var prefix = String.format("Swerve/Module%d/", this.constants.moduleNum);
            Logger.recordOutput(prefix + "driveSupCur", this.driveSupCur.refresh().getValue());
            Logger.recordOutput(prefix + "driveStaCur", this.driveStaCur.refresh().getValue());
            Logger.recordOutput(prefix + "angleOutCur", this.angleMotor.getOutputCurrent());
            Logger.recordOutput(prefix + "totalCur", this.driveSupCur.getValue() + this.angleMotor.getOutputCurrent());
        }
    }

    @Override
    public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
        layout.addDouble("Angle", () -> this.getAngle().getDegrees());
        layout.addDouble("Speed", this::getSpeed);
        layout.addDouble("Position", () -> this.getPosition().distanceMeters);
    }
}
