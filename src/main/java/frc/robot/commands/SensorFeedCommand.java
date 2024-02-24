package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class SensorFeedCommand extends Command {
    private boolean feederDetectedNote = false;
    // private Debouncer feederSensorDebouncer = new Debouncer(0.06);
    private IntakeSubsystem intake;
    private BooleanSupplier predicate;

    public SensorFeedCommand(IntakeSubsystem intake, BooleanSupplier predicate) {
        this.intake = intake;
        this.predicate = predicate;
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.feederDetectedNote = false;
    }

    @Override
    public void execute() {
        Logger.recordOutput("Intake/detectedNoteSensorFeed", this.feederDetectedNote);
        Logger.recordOutput("Intake/predicate", this.predicate.getAsBoolean());

        if (!this.predicate.getAsBoolean()) {
            this.feederDetectedNote = false;
            this.intake.setIntakeState(IntakeState.OFF);
            return;
        }

        if (!feederDetectedNote) {
            this.intake.setIntakeState(IntakeState.INTAKE);
        } else {
            this.intake.setIntakeState(IntakeState.OFF);
        }

        // if (feederSensorDebouncer.calculate(this.intake.getIndexSensor())) {
        if (this.intake.getIndexSensor()) {
            feederDetectedNote = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.setIntakeState(IntakeState.OFF);
    }
}
