package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class SensorFeedCommand extends Command {
    private boolean feederDetectedNote = false;
    // private Debouncer feederSensorDebouncer = new Debouncer(0.06);
    private IntakeSubsystem intake;

    public SensorFeedCommand(IntakeSubsystem intake) {
        this.intake = intake;
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.feederDetectedNote = false;
    }

    @Override
    public void execute() {
        Logger.recordOutput("Intake/detectedNoteSensorFeed", this.feederDetectedNote);

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
