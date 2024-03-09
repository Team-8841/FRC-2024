package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SimManager;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.sensors.imu.IMU;

// TODO: Make this use motorsafety

/**
 * Main swerve drive class, interfaces with a hardware IO class.
 */
public class DriveTrainSubsystem extends SubsystemBase {
    private final SwerveDriveKinematics kinematics;

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModuleIO swerveModules[];
    private SwerveModuleIOInputsAutoLogged autologgedInputs[];
    private IMU imu;

    private ChassisSpeeds setSpeed = new ChassisSpeeds();
    private PIDController omegaPID = new PIDController(0, 0, 0);
    private boolean useOmegaPID = false;

    @SuppressWarnings("all")
    public DriveTrainSubsystem(SwerveModuleIO[] swerveModules, IMU imu) {
        double baseRadius;
        if (Constants.isCompRobot) {
            this.kinematics = SwerveConstants.compSwerveKinematics;
            baseRadius = Math.hypot(SwerveConstants.compTrackWidth, SwerveConstants.compWheelBase);
        } 
        else {
            this.kinematics = SwerveConstants.driveBaseSwerveKinematics;
            baseRadius = Math.hypot(SwerveConstants.driveBaseTrackWidth, SwerveConstants.driveBaseWheelBase);
        }

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

        // this.swerveOdometry = new
        // SwerveDriveOdometry(SwerveConstants.swerveKinematics, imu.getHeading(),
        // this.getModulePositions());
        this.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, imu.getHeading(),
                this.getModulePositions(), new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(0)));

        if (RobotBase.isSimulation() && !Constants.simReplay) {
            SimManager.getInstance().registerDriveTrain(this::getPose, this::getSpeed);
        }

         // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        SwerveConstants.maxSpeed, // Max module speed, in m/s
                        baseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPosition(this.imu.getHeading(), this.getModulePositions(), pose);
    }

    private void initializeShuffleBoardWidgets() {
        ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");

        for (int i = 0; i < this.swerveModules.length; i++) {
            ShuffleboardLayout moduleLayout = robotTab.getLayout("Sweve Module " + i, BuiltInLayouts.kList);
            this.swerveModules[i].initializeShuffleBoardLayout(moduleLayout);
        }
    }

    private void driveInternal(ChassisSpeeds speeds) {
        var swerveModuleStates = this.kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (int i = 0; i < this.swerveModules.length; i++) {
            swerveModuleStates[i] = SwerveModuleState.optimize(swerveModuleStates[i], this.swerveModules[i].getAngle());

            this.swerveModules[i].setDesiredState(swerveModuleStates[i]);
            this.autologgedInputs[i].setAngle = swerveModuleStates[i].angle.getDegrees();
            this.autologgedInputs[i].setSpeedMetersPerSecond = swerveModuleStates[i].speedMetersPerSecond;
        }

        Logger.recordOutput("Swerve/setState", swerveModuleStates);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);
    }

    public void drive(ChassisSpeeds speeds, boolean useOmegaPID) {
        if (!this.useOmegaPID && useOmegaPID) {
            this.omegaPID.reset();
        }

        this.setSpeed = speeds;
        //this.setSpeed.vxMetersPerSecond *= -1;
        //this.setSpeed.vyMetersPerSecond *= -1;
        this.useOmegaPID = useOmegaPID;
    }

    public void drive(ChassisSpeeds speeds) {
        this.drive(speeds, false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean omegaPID) {
        this.drive(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        this.imu.getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation),
                omegaPID);
    }

    public ChassisSpeeds getSpeed() {
        return this.kinematics.toChassisSpeeds(this.getModuleStates());
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
        // Logging module states
        {
            var states = new SwerveModuleState[4];

            double totalCur = 0;
            for (int i = 0; i < this.swerveModules.length; i++) {
                SwerveModuleIO swerveModule = this.swerveModules[i];
                swerveModule.periodic();
                swerveModule.updateInputs(this.autologgedInputs[i]);
                Logger.processInputs("Swerve/Module" + i, this.autologgedInputs[i]);
                totalCur += swerveModule.getCurrent();
                states[i] = this.swerveModules[i].getState();
            }

            Logger.recordOutput("Swerve/actualStates", states);
            Logger.recordOutput("Swerve/totalCur", totalCur);
            this.poseEstimator.update(this.imu.getHeading(), this.getModulePositions());
            Logger.recordOutput("Swerve/poseOdometry", this.poseEstimator.getEstimatedPosition());
        }

        {
            if (DriverStation.isDisabled()) {
                this.setSpeed = new ChassisSpeeds();
            }

            // Speed logging
            var actualSpeed = this.getSpeed();

            Logger.recordOutput("Swerve/Speeds/set", this.setSpeed);
            Logger.recordOutput("Swerve/Speeds/actual", actualSpeed);
            Logger.recordOutput("Swerve/Speeds/error", this.setSpeed.minus(actualSpeed));

            // Omega PID
            double newOmega = this.setSpeed.omegaRadiansPerSecond * 1.0;
            if (this.useOmegaPID) {
                newOmega += this.omegaPID.calculate(actualSpeed.omegaRadiansPerSecond,
                        this.setSpeed.omegaRadiansPerSecond);
            }
            var newSpeed = new ChassisSpeeds(
                    this.setSpeed.vxMetersPerSecond,
                    this.setSpeed.vyMetersPerSecond,
                    newOmega);

            this.driveInternal(newSpeed);
        }
    }
}
