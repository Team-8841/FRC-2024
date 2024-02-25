package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.util.SafeCANSparkMax;
import frc.robot.constants.Constants;
import frc.robot.constants.elevator.ElevatorConstants;

public class RealElevatorIO implements ElevatorIO {
    private final CANSparkMax m_elevator = new CANSparkMax(ElevatorConstants.elevatorMain, MotorType.kBrushless),
            m_elevatorFollower = new CANSparkMax(ElevatorConstants.elevatorFollower,
                    MotorType.kBrushless);
    // 0 for brakes, 3 for shooter
    // off is extended

    private final Solenoid brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.brakeSolenoidPort);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(3), upperLimitSwitch = new DigitalInput(4);
    private final RelativeEncoder elevatorEncoder = this.m_elevator.getEncoder();

    private static final boolean BRAKE_DISENGAGE_VAL = false;
    private static final boolean BRAKE_ENGAGE_VAL = true;

    public RealElevatorIO() {
        this.m_elevatorFollower.follow(this.m_elevator, true);
        this.brakeSolenoid.set(BRAKE_DISENGAGE_VAL);
    }

    @Override
    public void set(double dcycle) {
        //if (!this.getLowerLimitSwitch() && !this.getUpperLimitSwitch() && !this.isBraking()) {
        if (!this.isBraking()) {
            this.m_elevator.set(dcycle);
        }
    }

    @Override
    public void setBrake(BrakeState state) {
        if (state == BrakeState.BRAKE_ENGAGE) {
            this.stopMotors();
        }

        this.brakeSolenoid.set(state == BrakeState.BRAKE_ENGAGE ? BRAKE_ENGAGE_VAL : BRAKE_DISENGAGE_VAL);
        this.stopMotors();
    }

    @Override
    public void stopMotors() {
        this.m_elevator.stopMotor();
    }

    @Override
    public boolean isBraking() {
        return this.brakeSolenoid.get() == BRAKE_ENGAGE_VAL;
    }

    @Override
    public boolean getUpperLimitSwitch() {
        return !this.upperLimitSwitch.get();
    }

    @Override
    public boolean getLowerLimitSwitch() {
        return !this.lowerLimitSwitch.get();
    }

    @Override
    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.setElevatorDCycle = this.m_elevator.get();
        inputs.actualElevatorRPS = this.elevatorEncoder.getVelocity() / 60;

        inputs.lowerLimitSwitch = this.lowerLimitSwitch.get();
        inputs.upperLimitSwitch = this.upperLimitSwitch.get();

        inputs.isBraking = this.isBraking();

        double elevatorOutCur = this.m_elevator.getOutputCurrent();
        Logger.recordOutput("Elevator/elevatorMotorOutCur", elevatorOutCur);
        double followerOutCur = this.m_elevatorFollower.getOutputCurrent();
        Logger.recordOutput("Elevator/elevatorFollowerMotorOutCur", followerOutCur);
        Logger.recordOutput("Elevator/totalCur", elevatorOutCur + followerOutCur);
    }
}
