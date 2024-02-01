package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CTREModuleState;
import frc.robot.SimManager;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.sensors.imu.IMU;

/**
 * Main swerve drive class, interfaces with a hardware IO class.
 */
public class DriveTrainSubsystem extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModuleIO swerveModules[];
    private SwerveModuleIOInputsAutoLogged autologgedInputs[];
    private IMU imu;

    @SuppressWarnings("all")
    public DriveTrainSubsystem(SwerveModuleIO[] swerveModules, IMU imu) {
        this.swerveModules = swerveModules;
        this.imu = imu;

        this.autologgedInputs = new SwerveModuleIOInputsAutoLogged[4];
        for (int i = 0; i < this.autologgedInputs.length; i++) {
            this.autologgedInputs[i] = new SwerveModuleIOInputsAutoLogged();
        }

        this.initializeShuffleBoardWidgets();

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1);
        this.resetModules();

        //this.swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, imu.getHeading(), this.getModulePositions());
        this.poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, imu.getHeading(), this.getModulePositions(), new Pose2d(new Translation2d(0, 0), Rotation2d.
            fromRadians(0)));

        if (RobotBase.isSimulation() && !Constants.simReplay) {
            SimManager.getInstance().registerDriveTrain(this::getPose, this::getSpeed);
        }
    }

    private void initializeShuffleBoardWidgets() {
        ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");

        for (int i = 0; i < this.swerveModules.length; i++) {
            ShuffleboardLayout moduleLayout = robotTab.getLayout("Sweve Module " + i, BuiltInLayouts.kList);
            this.swerveModules[i].initializeShuffleBoardLayout(moduleLayout);
        }
    }

    public void drive(ChassisSpeeds speeds) {
        var swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (int i = 0; i < this.swerveModules.length; i++) {
            swerveModuleStates[i] = SwerveModuleState.optimize(swerveModuleStates[i], this.swerveModules[i].getAngle());
            Logger.recordOutput("Swerve/Module" + i + "/state", swerveModuleStates[i]);

            this.swerveModules[i].setDesiredState(swerveModuleStates[i]);
            this.autologgedInputs[i].setAngle = swerveModuleStates[i].angle.getDegrees();
            this.autologgedInputs[i].setSpeedMetersPerSecond = swerveModuleStates[i].speedMetersPerSecond;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        this.drive(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        this.imu.getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
    }

    public ChassisSpeeds getSpeed() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return this.poseEstimator;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState states[] = new SwerveModuleState[4];

        for (int i = 0; i < this.swerveModules.length; i++) {
            states[i] = this.swerveModules[i].getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition positions[] = new SwerveModulePosition[4];

        for (int i = 0; i < this.swerveModules.length; i++) {
            positions[i] = this.swerveModules[i].getPosition();
        }

        return positions;
    }

    public void resetModules() {
        for (SwerveModuleIO swerveModule : this.swerveModules) {
            swerveModule.reset();
        }
    }

    public void setDrivePIDs(double kS, double kV, double kA, double kP, double kI, double kD) {
        for (var module : this.swerveModules) {
            module.setDrivePID(kS, kV, kA, kP, kI, kD);
        }
    }

    public void setSteeringPIDs(double kS, double kV, double kA, double kP, double kI, double kD) {
        for (var module : this.swerveModules) {
            module.setSteeringPID(kS, kV, kA, kP, kI, kD);
        }
    }

    @Override
    public void periodic() {
        double totalCur = 0;
        for (int i = 0; i < this.swerveModules.length; i++) {
            SwerveModuleIO swerveModule = this.swerveModules[i];
            swerveModule.periodic();
            swerveModule.updateInputs(this.autologgedInputs[i]);
            Logger.processInputs("Swerve/Module" + i, this.autologgedInputs[i]);
            totalCur += swerveModule.getCurrent();
        }

        Logger.recordOutput("Swerve/totalCur", totalCur);
        this.poseEstimator.update(this.imu.getHeading(), this.getModulePositions());
        Logger.recordOutput("Swerve/poseOdometry", this.poseEstimator.getEstimatedPosition());
    }
}
