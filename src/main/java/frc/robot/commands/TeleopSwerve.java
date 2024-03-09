package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class TeleopSwerve extends Command {
    private XboxController driveController;
    private DriveTrainSubsystem driveTrain;
    private VisionSubsystem vision;
    private PIDController visionPID = new PIDController(0.01, 0, 0);

    public TeleopSwerve(DriveTrainSubsystem driveTrain, XboxController driveController, VisionSubsystem vision) {
        this.driveTrain = driveTrain;
        this.driveController = driveController;
        this.vision = vision;

        this.addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        boolean visionAssist = this.driveController.getRightBumper();

        double forward = MathUtil.applyDeadband(-this.driveController.getLeftY(), Constants.controllerDeadband) * SwerveConstants.maxSpeed;
        double strafe;
        if (visionAssist) {
            strafe = this.visionPID.calculate(this.vision.getTargetHorizontalOffset());
        }
        else {
            strafe = MathUtil.applyDeadband(-this.driveController.getLeftX(), Constants.controllerDeadband) * SwerveConstants.maxSpeed;
        }
        double rotation = MathUtil.applyDeadband(-this.driveController.getRightX(), Constants.controllerDeadband)
                * AutoConstants.MaxAngularSpeedRadiansPerSecond;
        this.driveTrain.drive(new Translation2d(forward, strafe), rotation, !visionAssist, true);
    }
}
