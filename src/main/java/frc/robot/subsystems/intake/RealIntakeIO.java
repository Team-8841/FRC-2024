package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Intake.IntakeConstants;

public class RealIntakeIO implements IntakeIO {
    // Motors
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless),
            indexMotor = new CANSparkMax(IntakeConstants.indexMotor, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = this.intakeMotor.getEncoder(), indexEncoder = this.indexMotor.getEncoder();

    // Sensors
    private DigitalInput indexSenor = new DigitalInput(IntakeConstants.indexSensor);

    public RealIntakeIO() {
        this.intakeEncoder.setVelocityConversionFactor(1.0/60); // RPM to RPS
        this.indexEncoder.setVelocityConversionFactor(1.0/60); // RPM to RPS
    }

    @Override
    public void setIntakeSpeed(double dcycle) {
        this.intakeMotor.set(dcycle);
    }

    @Override
    public void setIndexSpeed(double dcycle) {
        this.indexMotor.set(dcycle);
    }

    @Override
    public boolean getIndexSensor() {
        // The sensor returns true when it detects something, and false otherwise
        return !this.indexSenor.get();
    }

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        inputs.setIntakeDCycle = this.intakeMotor.get();
        inputs.setIndexDCycle = this.indexMotor.get();
        inputs.actualIntakeRPS = this.indexEncoder.getVelocity();
        inputs.actualIndexRPS = this.indexEncoder.getVelocity();

        inputs.indexSensor = this.getIndexSensor();

        SmartDashboard.putBoolean("[Intake]: Note Sensor", this.getIndexSensor());

        Logger.recordOutput("Intake/intakeMotorOutCur", this.intakeMotor.getOutputCurrent());
        Logger.recordOutput("Intake/indexMotorOutCur", this.indexMotor.getOutputCurrent());
    }
}
