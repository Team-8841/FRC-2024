package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Intake.IntakeConstants;

public class RealIntakeIO implements IntakeIO {
    // Motors
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotor, MotorType.kBrushed),
            indexMotor = new CANSparkMax(IntakeConstants.indexMotor, MotorType.kBrushed);

    // Sensors
    private DigitalInput indexSenor = new DigitalInput(IntakeConstants.indexSensor);
    private RelativeEncoder intakeEncoder = this.intakeMotor.getEncoder(), indexEncoder = this.indexMotor.getEncoder();

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

    public boolean getIndexSensor() {
        return this.indexSenor.get();
    }

    public void updateInputs(IntakeInputsAutoLogged inputs) {
        inputs.actualIntakeRPS = this.indexEncoder.getVelocity();
        inputs.actualIndexRPS = this.indexEncoder.getVelocity();
        inputs.setIntakeDCycle = this.intakeMotor.get();
        inputs.setIndexDCycle = this.indexMotor.get();
    }
}
