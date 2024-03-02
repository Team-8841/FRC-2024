package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO.BrakeState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TeleopSwerve extends Command {
    // private DoubleSupplier forwardSupplier, strafeSupplier, rotationSupplier,
    // elevatorSupplier;
    // private Supplier<ShooterState> shooterStateSupplier;
    // private BooleanSupplier elevatorBrakeSupplier;
    private XboxController driveController;
    private Joystick copilotController;

    private DriveTrainSubsystem driveTrain;
    private ElevatorSubsystem elevator;

    // public TeleopSwerve(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter,
    // ElevatorSubsystem elevator,
    // DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier,
    // DoubleSupplier rotationSupplier, Supplier<ShooterState> shooterStateSupplier,
    // DoubleSupplier elevatorSupplier, BooleanSupplier elevatorBrakeSupplier) {
    // this.forwardSupplier = forwardSupplier;
    // this.strafeSupplier = strafeSupplier;
    // this.rotationSupplier = rotationSupplier;
    // this.shooterStateSupplier = shooterStateSupplier;
    // this.elevatorSupplier = elevatorSupplier;
    // this.elevatorBrakeSupplier = elevatorBrakeSupplier;
    // // this.strafeSupplier = () -> 0;
    // // this.rotationSupplier = strafeSupplier;
    // this.driveTrain = driveTrain;
    // this.shooter = shooter;
    // this.elevator = elevator;

    // this.addRequirements(driveTrain);
    // this.addRequirements(shooter);
    // this.addRequirements(elevator);
    // }

    public TeleopSwerve(DriveTrainSubsystem driveTrain, ElevatorSubsystem elevator,
            XboxController driveController, Joystick copilotController) {
        this.driveController = driveController;
        this.copilotController = copilotController;

        this.driveTrain = driveTrain;
        this.elevator = elevator;

        this.addRequirements(driveTrain);
        this.addRequirements(elevator);
    }

    @Override
    public void execute() {
        double forward = -this.driveController.getLeftY();
        double strafe = this.driveController.getLeftX();
        double rotation = this.driveController.getRightX();

        // Drivetrain
        Translation2d driveTranslation = new Translation2d(
                MathUtil.applyDeadband(forward, Constants.controllerDeadband) * SwerveConstants.maxSpeed,
                MathUtil.applyDeadband(strafe, Constants.controllerDeadband) * SwerveConstants.maxSpeed);
        rotation = MathUtil.applyDeadband(rotation, Constants.controllerDeadband)
                * AutoConstants.MaxAngularSpeedRadiansPerSecond;
        this.driveTrain.drive(driveTranslation, rotation, false, true);

        if (this.copilotController != null) {
            double elevatorPower = -this.copilotController.getX();
            boolean elevatorBrake = this.copilotController.getRawButton(7);

            // Elevator
            this.elevator.set(0.4 * MathUtil.applyDeadband(elevatorPower, 0.1));
            this.elevator.setBrake(elevatorBrake ? BrakeState.BRAKE_ENGAGE : BrakeState.BRAKE_DISENGAGE);
        }

        // Intake is handled as a binding to a trigger in `RobotContainer.java`
    }
}
