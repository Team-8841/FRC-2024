package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TeleopSwerve extends Command {
    private XboxController driveController;
    private CommandJoystick copilotController;
    private DriveTrainSubsystem driveTrain;
    private ShooterSubsystem shooter;
    private VisionSubsystem vision;
    private PIDController visionPID = new PIDController(0.01, 0, 0);

    private double getDSShooterPot() {

        double potVals[] = { 1, 0.55, 0.06, -0.44, -0.96 };
        double potRPM[] = { 0, 1500, 2000, 4000, 6000 };
        double epsilon = 0.1;
        double pot = this.copilotController.getRawAxis(3);

        // SmartDashboard.putNumber("[DS] pot", pot);

        for (int i = 0; i < potVals.length; i++) {
        if (potVals[i] - epsilon <= pot && potVals[i] + epsilon >= pot) {
            return potRPM[i];
        }
        }

        return 0;
    }


    public TeleopSwerve(DriveTrainSubsystem driveTrain, XboxController driveController, VisionSubsystem vision, ShooterSubsystem shooter, CommandJoystick joystick) {
        this.driveTrain = driveTrain;
        this.driveController = driveController;
        this.vision = vision;
        this.shooter = shooter;
        this.copilotController = joystick;

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

        var hid = this.driveController;
        this.shooter.setShooterSpeed(this.getDSShooterPot());
        this.shooter.setShooterAngle(!hid.getLeftBumper());
    }
}
