package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.swerve.PureTalonFXConstants;
import frc.robot.constants.swerve.SwerveConstants;

public class TalonFXSwerveModuleIO implements SwerveModuleIO {
  private TalonFX driveMotor, steeringMotor;
  private CANcoder steeringEncoder;

  private Rotation2d lastAngle;

  private SwerveModuleConstants constants;

  private final VelocityDutyCycle driveVelVoltage = new VelocityDutyCycle(0).withSlot(0);
  private final PositionDutyCycle steeringPosVoltage = new PositionDutyCycle(0).withSlot(0);
  private final StatusSignal<Double> driveVel, drivePos, driveSupCur, driveStaCur, steeringPos, steeringSupCur, steeringStaCur;

  /**
   * Creates a new container from each of the components.
   *
   * @param driveMotor The TalonFX controller controlling the drive motor.
   * @param steeringMotor The SparkMax controller controlling the steering motor.
   * @param steeringEncoder The CTRE CANCoder attached to the steering motor.
   * @param constants Module specific constants.
   */
  public TalonFXSwerveModuleIO(
      TalonFX driveMotor,
      TalonFX steeringMotor,
      CANcoder steeringEncoder,
      SwerveModuleConstants constants) {
    this.driveMotor = driveMotor;
    this.steeringMotor = steeringMotor;
    this.steeringEncoder = steeringEncoder;
    this.constants = constants;

    this.configSteeringEncoder();
    this.configDriveMotor();
    this.configSteeringMotor();

    this.driveVel = this.driveMotor.getVelocity();
    this.drivePos = this.driveMotor.getPosition();
    this.driveSupCur = this.driveMotor.getSupplyCurrent();
    this.driveStaCur = this.driveMotor.getStatorCurrent();
    this.steeringPos = this.steeringEncoder.getAbsolutePosition();
    this.steeringSupCur = this.steeringMotor.getSupplyCurrent();
    this.steeringStaCur = this.steeringMotor.getStatorCurrent();
  }

  /**
   * Creates a new container from each of the component's CAN ID's.
   *
   * @param constants Module specific constants.
   */
  public TalonFXSwerveModuleIO(SwerveModuleConstants constants) {
    this(
        new TalonFX(constants.driveMotorID),
        new TalonFX(constants.angleMotorID),
        new CANcoder(constants.cancoderID),
        constants);
  }

  private void configSteeringEncoder() {
    CANcoderConfigurator configurator = this.steeringEncoder.getConfigurator();
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    magnetSensorConfigs.SensorDirection = SwerveConstants.canCoderDir;
    magnetSensorConfigs.MagnetOffset = -this.constants.angleOffset.getRotations();
    configurator.apply(magnetSensorConfigs);
  }

  private void configDriveMotor() {
    this.driveMotor.getConfigurator().apply(PureTalonFXConstants.driveMotorConfigs);
  }

  private void configSteeringMotor() {
    TalonFXConfigurator configurator = this.steeringMotor.getConfigurator();
    configurator.apply(PureTalonFXConstants.angleMotorConfigs);

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.FeedbackRemoteSensorID = this.constants.cancoderID;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    feedbackConfigs.RotorToSensorRatio = SwerveConstants.angleGearRatio;
    configurator.apply(feedbackConfigs);
  }

  @Override
  public void setSpeed(SwerveModuleState desiredState) {
    // Closed loop
    double velocity =
        Conversions.metersToRots(
            desiredState.speedMetersPerSecond,
            SwerveConstants.wheelCircumference,
            SwerveConstants.driveGearRatio);
    this.driveMotor.setControl(this.driveVelVoltage.withVelocity(velocity));
  }

  @Override
  public void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    this.lastAngle = angle;

    this.steeringMotor.setControl(this.steeringPosVoltage.withPosition(angle.getRotations()));
  }

  @Override
  public SwerveModuleState getState() {
    double angle = this.steeringPos.refresh().getValue();

    return new SwerveModuleState(
        Conversions.rotsToMeters(
            this.driveVel.refresh().getValue(),
            SwerveConstants.wheelCircumference,
            SwerveConstants.driveGearRatio),
        Rotation2d.fromRotations(angle));
  }

  @Override
  public SwerveModulePosition getPosition() {
    double angle = this.steeringPos.refresh().getValue();

    return new SwerveModulePosition(
        Conversions.rotsToMeters(
            this.drivePos.refresh().getValue(),
            SwerveConstants.wheelCircumference,
            SwerveConstants.driveGearRatio),
        Rotation2d.fromRotations(angle));
  }

  @Override
  public double getCurrent() {
    return this.driveSupCur.getValue() + this.steeringSupCur.getValue();
  }

  @Override
  public void periodic() {
    var prefix = String.format("/Swerve/Module%d/", this.constants.moduleNum);
    Logger.recordOutput(prefix + "driveSupCur", this.driveSupCur.refresh().getValue());
    Logger.recordOutput(prefix + "driveStaCur", this.driveStaCur.refresh().getValue());
    Logger.recordOutput(prefix + "angleSupCur", this.steeringSupCur.refresh().getValue());
    Logger.recordOutput(prefix + "angleStaCur", this.steeringStaCur.refresh().getValue());
    Logger.recordOutput(prefix + "totalCur", this.driveSupCur.getValue() + this.steeringSupCur.getValue());
  }

  @Override
  public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
    layout.addDouble("Angle", () -> this.getAngle().getDegrees());
    layout.addDouble("Speed", this::getSpeed);
    layout.addDouble("Position", () -> this.getPosition().distanceMeters);
  }
}
