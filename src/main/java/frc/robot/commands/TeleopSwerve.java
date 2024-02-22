package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;

public class TeleopSwerve extends Command {
    private DoubleSupplier forwardSupplier, strafeSupplier, rotationSupplier;
    private Supplier<ShooterState> shooterStateSupplier;
    private DriveTrainSubsystem driveTrain;
    private ShooterSubsystem shooter;

    public TeleopSwerve(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier, Supplier<ShooterState> shooterStateSupplier) {
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.shooterStateSupplier = shooterStateSupplier;
        // this.strafeSupplier = () -> 0;
        // this.rotationSupplier = strafeSupplier;
        this.driveTrain = driveTrain;
        this.shooter = shooter;

        this.addRequirements(driveTrain);
        this.addRequirements(shooter);
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
    }
}
