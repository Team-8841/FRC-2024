package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.util.SafeCANSparkMax;
import frc.robot.constants.Intake.IntakeConstants;

public class RealIntakeIO implements IntakeIO {
    // Motors
    private final SafeCANSparkMax intakeMotor = new SafeCANSparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless),
            indexMotor = new SafeCANSparkMax(IntakeConstants.indexMotor, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = this.intakeMotor.getEncoder(), indexEncoder = this.indexMotor.getEncoder();

    // Sensors
    private DigitalInput indexSenor = new DigitalInput(IntakeConstants.indexSensor), intakeSensor = new DigitalInput(IntakeConstants.intakeSensor);

    public RealIntakeIO() {
        this.intakeEncoder.setVelocityConversionFactor(1.0/60); // RPM to RPS
        this.indexEncoder.setVelocityConversionFactor(1.0/60); // RPM to RPS
    }

    public void setIntakeSpeed(double dcycle) {
        this.intakeMotor.set(dcycle);
    }

    public void setIndexSpeed(double dcycle) {
        this.indexMotor.set(dcycle);
    }

    public void feed() {
        this.intakeMotor.feed();
        this.indexMotor.feed();
    }

    public boolean getIntakeSensor() {
        return this.intakeSensor.get();
    }

    public boolean getIndexSensor() {
        return this.indexSenor.get();
    }

    public void updateInputs(IntakeInputsAutoLogged inputs) {
        inputs.setIntakeDCycle = this.intakeMotor.get();
        inputs.setIndexDCycle = this.indexMotor.get();
        inputs.actualIntakeRPS = this.indexEncoder.getVelocity();
        inputs.actualIndexRPS = this.indexEncoder.getVelocity();

        inputs.intakeSensor = this.getIntakeSensor();
        inputs.indexSensor = this.getIndexSensor();

        inputs.intakeAlive = this.intakeMotor.isAlive();
        inputs.indexAlive = this.indexMotor.isAlive();

        Logger.recordOutput("Intake/intakeMotorOutCur", this.intakeMotor.getOutputCurrent());
        Logger.recordOutput("Intake/indexMotorOutCur", this.indexMotor.getOutputCurrent());
    }
}
