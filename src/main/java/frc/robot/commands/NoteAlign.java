package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import javax.swing.text.html.Option;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.sensors.vision.piecetracking.LimelightTracker;

public class NoteAlign extends Command {
    private PIDController transPID = new PIDController(VisionConstants.noteAlign_kP, VisionConstants.noteAlign_kI,
            VisionConstants.noteAlign_kD);
    private DoubleConsumer transConsumer;
    private LimelightTracker tracker;

    public NoteAlign(LimelightTracker tracker, DoubleConsumer transConsumer) {
        this.transPID.setTolerance(1);
        this.transConsumer = transConsumer;
        this.tracker = tracker;
    }

    @Override
    public void initialize() {
        this.tracker.schedule();
    }

    @Override
    public void execute() {
        this.tracker.getTargetAngle().ifPresentOrElse(
                ((angle) -> this.transConsumer.accept(this.transPID.calculate(angle.getDegrees()))),
                () -> this.transConsumer.accept(0));
    }

    @Override
    public void end(boolean isInterrupted) {
        this.tracker.cancel();
    }

    @Override
    public boolean isFinished() {
        return this.transPID.atSetpoint();
    }
}
