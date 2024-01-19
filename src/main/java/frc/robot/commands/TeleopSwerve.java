package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class TeleopSwerve extends Command {
    private DoubleSupplier forwardSupplier, strafeSupplier, rotationSupplier, shooterPowerSupplier;
    private DriveTrainSubsystem driveTrain;

    public TalonFX mainShooterMotor = new TalonFX(Constants.mainShooterId), followerShooterMotor = new TalonFX(Constants.followerShooterId);

    public TeleopSwerve(DriveTrainSubsystem driveTrain, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier shooterPowerSupplier) {
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.shooterPowerSupplier = shooterPowerSupplier;
        // this.strafeSupplier = () -> 0;
        // this.rotationSupplier = strafeSupplier;
        this.driveTrain = driveTrain;

        this.mainShooterMotor.setControl(new VoltageOut(0));
        this.followerShooterMotor.setControl(new Follower(Constants.mainShooterId, false));

        this.addRequirements(driveTrain);
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

        this.mainShooterMotor.setControl(new VoltageOut(12 * this.shooterPowerSupplier.getAsDouble()));
    }
}
