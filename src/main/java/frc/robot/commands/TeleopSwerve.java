package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO.BrakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;

public class TeleopSwerve extends Command {
    private DoubleSupplier forwardSupplier, strafeSupplier, rotationSupplier, elevatorSupplier;
    private Supplier<ShooterState> shooterStateSupplier;
    private BooleanSupplier elevatorBrakeSupplier;

    private DriveTrainSubsystem driveTrain;
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;

    public TeleopSwerve(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, ElevatorSubsystem elevator, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier, Supplier<ShooterState> shooterStateSupplier, DoubleSupplier elevatorSupplier, BooleanSupplier elevatorBrakeSupplier) {
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.shooterStateSupplier = shooterStateSupplier;
        this.elevatorSupplier = elevatorSupplier;
        this.elevatorBrakeSupplier = elevatorBrakeSupplier;
        // this.strafeSupplier = () -> 0;
        // this.rotationSupplier = strafeSupplier;
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.elevator = elevator;

        this.addRequirements(driveTrain);
        this.addRequirements(shooter);
        this.addRequirements(elevator);
    }

    @Override
    public void execute() {
        Translation2d driveTranslation = new Translation2d(
                MathUtil.applyDeadband(forwardSupplier.getAsDouble(), Constants.controllerDeadband)
                        * SwerveConstants.maxSpeed,
                MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.controllerDeadband)
                        * SwerveConstants.maxSpeed);
        double rotation = MathUtil.applyDeadband(this.rotationSupplier.getAsDouble(), Constants.controllerDeadband)
                * AutoConstants.MaxAngularSpeedRadiansPerSecond;

        this.driveTrain.drive(driveTranslation, rotation, true);

        this.shooter.setShooterState(this.shooterStateSupplier.get());

        this.elevator.set(0.4 * MathUtil.applyDeadband(this.elevatorSupplier.getAsDouble(), 0.1));

        this.elevator.setBrake(this.elevatorBrakeSupplier.getAsBoolean() ? BrakeState.BRAKE_ENGAGE : BrakeState.BRAKE_DISENGAGE);
    }
}
