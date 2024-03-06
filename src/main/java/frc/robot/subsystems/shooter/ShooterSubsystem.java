package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Constants.CandleConstants;
import frc.robot.subsystems.LEDSubsystem;

public class ShooterSubsystem extends SubsystemBase {
    private DigitalInput EEInSensor = new DigitalInput(1);
    private DigitalInput EEOutSensor = new DigitalInput(2);

    private CANSparkMax endEffector = new CANSparkMax(ShooterConstants.endEffectorROTID, MotorType.kBrushless);
    private CANSparkMax endEffectorRoller = new CANSparkMax(19, MotorType.kBrushless);
    private RelativeEncoder endEffectorEncoder = this.endEffector.getEncoder();

    private PIDController EEPIDController = new PIDController(0.013, 0, 0);

    private HoodPosition setPosition = HoodPosition.STOWED;

    private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID);
    private TalonFX followerMotor = new TalonFX(ShooterConstants.followerMotorID);

    private Solenoid angleSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    private LEDSubsystem leds;

    private LinearFilter rpmFilter = LinearFilter.movingAverage(5);
    private double shooterSetRPS, filteredRPM;
    private boolean shooterGotToSpeed;

    private static final boolean SHOOTER_UP = false;
    private static final boolean SHOOTER_DOWN = true;

    public static enum HoodPosition {
        STOWED(Rotation2d.fromDegrees(0)), DEPLOYED(Rotation2d.fromDegrees(0)); // TODO: Needs to be tuned

        Rotation2d hoodAngle;

        private HoodPosition(Rotation2d hoodAngle) {
            this.hoodAngle = hoodAngle;
        }
    }

    public ShooterSubsystem(LEDSubsystem leds) {
        /* Shooter */

        this.shooterMotor.getConfigurator().apply(ShooterConstants.shooterConfig);
        this.followerMotor.getConfigurator().apply(ShooterConstants.shooterConfig);

        shooterMotor.setInverted(true);
        followerMotor.setControl(new Follower(shooterMotor.getDeviceID(), false));

        /* End Effector */
        endEffector.restoreFactoryDefaults();
        endEffector.setSmartCurrentLimit(10);
        endEffector.setIdleMode(IdleMode.kBrake);
        endEffector.setInverted(true);

        // 1:100 planetary gear reduction, then 16:36 sprocket gear reduction
        endEffectorEncoder.setPositionConversionFactor(1.0 / (100.0 * 36.0 / 16.0));

        endEffectorRoller.restoreFactoryDefaults();
        endEffectorRoller.setSmartCurrentLimit(10);
        endEffectorRoller.setIdleMode(IdleMode.kCoast);
        endEffectorRoller.setInverted(true);

        this.leds = leds;
    }

    public ShooterSubsystem() {
        this(null);
    }

    @Override
    public void periodic() {
        // Logging
        {
            Logger.recordOutput("Hood/EE/ActualDeg", getEEAngle().getDegrees());
            Logger.recordOutput("Hood/EE/SetDeg", this.setPosition.hoodAngle.getDegrees());
        }

        // Endeffector PID loop
        {
            double pidEffort = EEPIDController.calculate(getEEAngle().getDegrees(),
                    this.setPosition.hoodAngle.getDegrees());
            Logger.recordOutput("Hood/EE/PIDEffort", pidEffort);

            if (getEEHomeLimit()) {
                zeroEEAngle();
            }

            if (getEEHomeLimit() && pidEffort < 0) {
                endEffector.set(0);
            } else if (getEEDeployedLimit() && pidEffort > 0) {
                endEffector.set(0);
            } else {
                endEffector.set(pidEffort);
            }
        }

        // Shooter
        {
            this.filteredRPM = this.rpmFilter.calculate(this.shooterMotor.getVelocity().getValueAsDouble() * 60);

            if (!this.shooterGotToSpeed && this.isShooterAtSpeed()) {
                // Strobe when shooter gets up to speed
                this.leds.animate(new StrobeAnimation(0, 255, 0, 0, 1, CandleConstants.kLEDCount), 1).schedule();
                this.shooterGotToSpeed = true;
            }
        }
    }

    public void setEERoller(double speed) {
        endEffectorRoller.set(speed);
    }

    public void setHoodPosition(HoodPosition position) {
        this.setPosition = position;
    }

    public Rotation2d getEEAngle() {
        return Rotation2d.fromRotations(endEffector.getEncoder().getPosition());
    }

    public boolean getEEHomeLimit() {
        return !EEInSensor.get();
    }

    public boolean getEEDeployedLimit() {
        return !EEOutSensor.get();
    }

    public void zeroEEAngle() {
        endEffector.getEncoder().setPosition(0);
    }

    public void setShooterSpeed(double rpm) {
        // velocity is set in RPS
        this.shooterSetRPS = rpm / 60;
        this.shooterMotor.setControl(new VelocityVoltage(rpm / 60.0).withSlot(0));

        this.shooterGotToSpeed = this.isShooterAtSpeed();
    }

    public double getShooterRPM() {
        return this.filteredRPM;
    }

    public boolean isShooterAtSpeed() {
        return this.filteredRPM > 60 * this.shooterSetRPS - 100;
    }

    public void setShooterAngle(boolean isUp) {
        angleSolenoid.set(isUp ? SHOOTER_UP : SHOOTER_DOWN);
    }

    public boolean isShooterUp() {
        return angleSolenoid.get() == SHOOTER_UP;
    }

    private void updateStatus() {
        SmartDashboard.putNumber("[Shooter]: Setpoint (RPM)", this.shooterSetRPS * 60);
        SmartDashboard.putNumber("[Shooter]: Velocity (RPM)", getShooterRPM());
        SmartDashboard.putBoolean("[Shooter]: Is Shooter Up", isShooterUp());

        Logger.recordOutput("Shooter/actualShooterRPM", this.shooterMotor.getVelocity().getValueAsDouble() * 60);
        Logger.recordOutput("Shooter/filteredShooterRPM", this.filteredRPM);
        Logger.recordOutput("Shooter/setShooterRPM", this.shooterSetRPS);
        Logger.recordOutput("Shooter/isShooterUp", this.isShooterUp());
    }
}
