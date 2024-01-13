package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private DoubleSupplier forwardSupplier, strafeSupplier, rotationSupplier;
  private DriveTrainSubsystem driveTrain;

  public TeleopSwerve(
      DriveTrainSubsystem driveTrain,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier) {
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    // this.strafeSupplier = () -> 0;
    // this.rotationSupplier = strafeSupplier;
    this.driveTrain = driveTrain;

    this.addRequirements(driveTrain);
  }

  @Override
  public void execute() {
    Translation2d driveTranslation =
        new Translation2d(
            MathUtil.applyDeadband(forwardSupplier.getAsDouble(), Constants.controllerDeadband)
                * SwerveConstants.maxSpeed,
            MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.controllerDeadband)
                * SwerveConstants.maxSpeed);
    double rotation =
        MathUtil.applyDeadband(this.rotationSupplier.getAsDouble(), Constants.controllerDeadband)
            * AutoConstants.MaxAngularSpeedRadiansPerSecond;

    this.driveTrain.drive(driveTranslation, rotation, false);
  }
}
