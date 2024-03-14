package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.constants.Constants.CandleConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem;

public class ShooterSubsystem extends SubsystemBase {
    private DigitalInput EEInSensor = new DigitalInput(1);
    private DigitalInput EEOutSensor = new DigitalInput(2);

    private CANSparkMax endEffector = new CANSparkMax(ShooterConstants.endEffectorROTID, MotorType.kBrushless);
    private CANSparkMax endEffectorRoller = new CANSparkMax(19, MotorType.kBrushless);
    private RelativeEncoder endEffectorEncoder = this.endEffector.getEncoder();

    private PIDController EEPIDController = new PIDController(0.005, 0, 0);

    // private HoodPosition setPosition = HoodPosition.STOWED;
    private boolean isHoodSetUp = false, hoodInited = false;

    private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID);
    private TalonFX followerMotor = new TalonFX(ShooterConstants.followerMotorID);

    private Solenoid angleSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    private LEDSubsystem leds;

    private LinearFilter rpmFilter = LinearFilter.movingAverage(5);
    private double shooterSetRPS, filteredRPM;
    private boolean shooterGotToSpeed;

    private static final boolean SHOOTER_UP = false;
    private static final boolean SHOOTER_DOWN = true;

    // public static enum HoodPosition {
    //     STOWED(Rotation2d.fromDegrees(0)), DEPLOYED(Rotation2d.fromDegrees(105)); // TODO: Needs to be tuned

    //     Rotation2d hoodAngle;

    //     private HoodPosition(Rotation2d hoodAngle) {
    //         this.hoodAngle = hoodAngle;
    //     }
    // }

    public ShooterSubsystem(LEDSubsystem leds) {
        /* Shooter */

        this.shooterMotor.getConfigurator().apply(ShooterConstants.shooterConfig);
        this.followerMotor.getConfigurator().apply(ShooterConstants.shooterConfig);

        shooterMotor.setInverted(true);
        followerMotor.setControl(new Follower(shooterMotor.getDeviceID(), false));

        /* End Effector */
        endEffector.restoreFactoryDefaults();
        endEffector.setSmartCurrentLimit(30);
        endEffector.setIdleMode(IdleMode.kBrake);
        endEffector.setInverted(false);

        // 1:100 planetary gear reduction, then 16:36 sprocket gear reduction
        endEffectorEncoder.setPositionConversionFactor(1.0 / (100.0 * 36.0 / 16.0));

        endEffectorRoller.restoreFactoryDefaults();
        endEffectorRoller.setSmartCurrentLimit(30);
        endEffectorRoller.setIdleMode(IdleMode.kCoast);
        endEffectorRoller.setInverted(true);

        this.leds = leds;

        Shuffleboard.getTab("Robot").addDouble("Shooter RPM", this::getShooterRPM);
        Shuffleboard.getTab("Robot").addDouble("Set Shooter RPM", this::getShooterRPM);
        Shuffleboard.getTab("Robot").addBoolean("EE Home Limit", this::getEEHomeLimit);
        Shuffleboard.getTab("Robot").addBoolean("EE Deployed Limit", this::getEEDeployedLimit);
    }

    public ShooterSubsystem() {
        this(null);
    }

    @Override
    public void periodic() {
        // Logging
        {
            Logger.recordOutput("Hood/EE/ActualDeg", getEEAngle().getDegrees());
        }

        // Endeffector PID loop
        if (this.hoodInited) {
            {
                boolean homeLimit = this.getEEHomeLimit();
                boolean deployedLimit = this.getEEDeployedLimit();

                if (homeLimit && deployedLimit) {
                    this.endEffector.set(0);
                }
                else if (homeLimit && this.isHoodSetUp) {
                    this.endEffector.set(-0.4);
                }
                else if (deployedLimit && !this.isHoodSetUp) {
                    this.endEffector.set(0.4);
                }
                else if (!homeLimit && !deployedLimit) {
                    if (this.isHoodSetUp) {
                        this.endEffector.set(-0.4);
                    }
                    else {
                        this.endEffector.set(0.4);
                    }
                }
                else if (homeLimit || deployedLimit) {
                    this.endEffector.set(0);
                }
            }
            // else {
            //     this.endEffector.set(0);
            // }
        }

        // Shooter
        {
            this.filteredRPM = this.rpmFilter.calculate(this.shooterMotor.getVelocity().getValueAsDouble() * 60);

            // if (this.leds != null && !this.shooterGotToSpeed && this.isShooterAtSP() && 60 * this.shooterSetRPS >= 250) {
            //     // Strobe when shooter gets up to speed
            //     this.leds.animate(new StrobeAnimation(0x66, 0xff, 0x33, 0, 0.33, CandleConstants.kLEDCount), 1).schedule();
            //     this.shooterGotToSpeed = true;
            // }
        }

        this.updateStatus();
    }

    public Command initHoodCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                this.endEffector.set(0);
            }, 
            (interrupted) -> {
                // Stop moving it back ofc
                this.endEffector.set(0);
                // We initialized the hood
                this.hoodInited = !interrupted;
                // Zero angle
                this.endEffectorEncoder.setPosition(0);
            }, 
            () -> this.getEEHomeLimit() || this.hoodInited, 
            this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        // return null;
    }

    public void setEERoller(double speed) {
        endEffectorRoller.set(speed);
    }

    public void setHood(boolean isHoodUp) {
        this.isHoodSetUp = isHoodUp;
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

    public void setShooterSpeed(double rpm) {
        // velocity is set in RPS
        this.shooterSetRPS = rpm / 60;
        this.shooterMotor.setControl(new VelocityVoltage(rpm / 60.0).withSlot(0));

        this.shooterGotToSpeed = this.isShooterAtSP();
    }

    public double getShooterRPM() {
        return this.filteredRPM;
    }

    public double getShooterSPRPM() {
        return this.shooterSetRPS * 60;
    }

    public boolean isShooterAtSP() {
        //return this.filteredRPM > 60 * this.shooterSetRPS - 100;
        return Math.abs(this.filteredRPM - 60 * this.shooterSetRPS) <= 200.0;
    }

    public boolean isHoodAtSP() {
        return this.isHoodSetUp ? this.getEEDeployedLimit() : this.getEEHomeLimit();
    }

    public boolean isHoodSetUp() {
        return this.isHoodSetUp;
    }

    public void setShooterAngle(boolean isUp) {

        angleSolenoid.set(isUp ? SHOOTER_UP : SHOOTER_DOWN);
    }

    public boolean isShooterUp() {
        return angleSolenoid.get() == SHOOTER_UP;
    }

    private void updateStatus() {
        SmartDashboard.putNumber("[Shooter]: Setpoint (RPM)", this.shooterSetRPS * 60);
        SmartDashboard.putNumber("[Shooter]: Velocity (RPM)", this.getShooterRPM());
        SmartDashboard.putBoolean("[Shooter]: Is Shooter Up", this.isShooterUp());

        Logger.recordOutput("Shooter/actualShooterRPM", this.shooterMotor.getVelocity().getValueAsDouble() * 60);
        Logger.recordOutput("Shooter/filteredShooterRPM", this.filteredRPM);
        Logger.recordOutput("Shooter/setShooterRPM", this.shooterSetRPS * 60);
        Logger.recordOutput("Shooter/isShooterUp", this.isShooterUp());
        Logger.recordOutput("Shooter/shooterGotToSpeed", this.shooterGotToSpeed);
        Logger.recordOutput("Shooter/shooterAtSP", this.isShooterAtSP());

        Logger.recordOutput("Shooter/hoodSetUp", this.isHoodSetUp);
        Logger.recordOutput("Shooter/hoodAtSP", this.isHoodAtSP());
    }
}
