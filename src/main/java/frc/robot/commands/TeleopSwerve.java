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
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO.BrakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;

public class TeleopSwerve extends Command {
    // private DoubleSupplier forwardSupplier, strafeSupplier, rotationSupplier,
    // elevatorSupplier;
    // private Supplier<ShooterState> shooterStateSupplier;
    // private BooleanSupplier elevatorBrakeSupplier;
    private XboxController driveController;
    private Joystick copilotController;

    private DriveTrainSubsystem driveTrain;
    private ShooterSubsystem shooter;
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

    public TeleopSwerve(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, ElevatorSubsystem elevator,
            XboxController driveController, Joystick copilotController) {
        this.driveController = driveController;
        this.copilotController = copilotController;

        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.elevator = elevator;

        this.addRequirements(driveTrain);
        this.addRequirements(shooter);
        this.addRequirements(elevator);
    }

    private ShooterState getShooterState() {
        if (this.driveController.getLeftBumper()) {
            if (this.driveController.getRightBumper()) {
                return ShooterState.AMPSHOT;
            }
            return ShooterState.AMPRAISE;
        }

        double potVals[] = { 1, 0.55, 0.06, -0.44, -0.96 };
        ShooterState potStates[] = { ShooterState.OFF, ShooterState.COOLSHOT, ShooterState.COOLSHOT,
                ShooterState.COOLSHOT,
                ShooterState.COOLSHOT };
        double epsilon = 0.1;
        double pot = this.copilotController.getRawAxis(3);

        for (int i = 0; i < potVals.length; i++) {
            if (potVals[i] - epsilon <= pot && potVals[i] + epsilon >= pot) {
                return potStates[i];
            }
        }

        return ShooterState.OFF;
    }

    @Override
    public void execute() {
        double forward = -this.driveController.getLeftY();
        double strafe = this.driveController.getLeftX();
        double rotation = this.driveController.getRightX();
        double elevatorPower = -this.copilotController.getX();
        boolean elevatorBrake = this.copilotController.getRawButton(7);

        // Drivetrain
        Translation2d driveTranslation = new Translation2d(
                MathUtil.applyDeadband(forward, Constants.controllerDeadband) * SwerveConstants.maxSpeed,
                MathUtil.applyDeadband(strafe, Constants.controllerDeadband) * SwerveConstants.maxSpeed);
        rotation = MathUtil.applyDeadband(rotation, Constants.controllerDeadband)
                * AutoConstants.MaxAngularSpeedRadiansPerSecond;
        this.driveTrain.drive(driveTranslation, rotation, true, true);

        // Shooter
        this.shooter.setShooterState(this.getShooterState());

        // Elevator
        this.elevator.set(0.4 * MathUtil.applyDeadband(elevatorPower, 0.1));
        this.elevator.setBrake(elevatorBrake ? BrakeState.BRAKE_ENGAGE : BrakeState.BRAKE_DISENGAGE);

        // Intake is handled as a binding to a trigger in `RobotContainer.java`
    }
}
