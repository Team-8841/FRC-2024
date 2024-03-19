// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.RainbowAnimation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.SensorFeedCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.swerve.BaseRobotConstants;
import frc.robot.constants.swerve.CompRobotConstants;
import frc.robot.sensors.imu.DummyIMU;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.NavX2;
import frc.robot.sensors.imu.SimIMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.drive.DummySwerveModuleIO;
import frc.robot.subsystems.drive.MixedSwerveModuleIO;
import frc.robot.subsystems.drive.SimSwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.TalonFXSwerveModuleIO;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.DummyIntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem driveTrain;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ElevatorSubsystem elevator;
  private VisionSubsystem vision;

  private LEDSubsystem led = new LEDSubsystem(22);

  // Controllers
  private CommandXboxController driveController;
  private CommandJoystick copilotController;

  /* Copilot Buttons */
  private JoystickButton compressorOverrideBtn;
  private JoystickButton hoodDeployBtn;
  private JoystickButton hoodOverrideBtn;
  private JoystickButton intakeInBtn;
  private JoystickButton intakeOutBtn;
  private JoystickButton shooterAngleBtn;
  private JoystickButton climberBreaksBtn;

  /* Driver Buttons */
  private JoystickButton shootBtn;
  private JoystickButton visionAssistBtn;
  private JoystickButton zeroGyroBtn;
  private double hoodTrigger;

  private Command shooterCommand;

  private double climberMovement;
  public SendableChooser<Command> m_AutoChooser;

  public RobotContainer(CommandXboxController driveController, CommandJoystick copilotController) {
    this.driveController = driveController;
    this.copilotController = copilotController;

    if (copilotController != null) {
      compressorOverrideBtn = new JoystickButton(copilotController.getHID(), OIConstants.button1);
      hoodDeployBtn = new JoystickButton(copilotController.getHID(), OIConstants.button2);
      hoodOverrideBtn = new JoystickButton(copilotController.getHID(), OIConstants.button3);
      intakeInBtn = new JoystickButton(copilotController.getHID(), OIConstants.button4);
      intakeOutBtn = new JoystickButton(copilotController.getHID(), OIConstants.button5);
      shooterAngleBtn = new JoystickButton(copilotController.getHID(), OIConstants.button6);
      climberBreaksBtn = new JoystickButton(copilotController.getHID(), OIConstants.button7);
      climberMovement = copilotController.getHID().getRawAxis(OIConstants.analog1);
    }

    SwerveModuleIO swerveModules[];
    if (RobotBase.isReal()) {
      // Real robot
      this.imu = new NavX2();
      if (Constants.isCompRobot) {
        swerveModules = new SwerveModuleIO[] {
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod0.constants, false),
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod1.constants, false),
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod2.constants, false),
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod3.constants, false),
        };

        this.intake = new IntakeSubsystem(new RealIntakeIO(), this.led);
        this.shooter = new ShooterSubsystem(this.led);
        this.elevator = new ElevatorSubsystem();
        this.vision = new VisionSubsystem();
      } else {
        swerveModules = new SwerveModuleIO[] {
            new MixedSwerveModuleIO(BaseRobotConstants.Mod1.constants, false),
            new MixedSwerveModuleIO(BaseRobotConstants.Mod0.constants, false),
            new MixedSwerveModuleIO(BaseRobotConstants.Mod3.constants, false),
            new MixedSwerveModuleIO(BaseRobotConstants.Mod2.constants, false),
        };
      }
    } else if (Constants.simReplay) {
      // Replay
      swerveModules = new SwerveModuleIO[] {
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
      };

      this.imu = new DummyIMU();

      this.intake = new IntakeSubsystem(new DummyIntakeIO(), this.led);
    } else {
      // Physics sim
      swerveModules = new SwerveModuleIO[] {
          new SimSwerveModuleIO(),
          new SimSwerveModuleIO(),
          new SimSwerveModuleIO(),
          new SimSwerveModuleIO(),
      };

      this.imu = new SimIMU();
    }

    this.driveTrain = new DriveTrainSubsystem(swerveModules, this.imu);

    this.imu.initializeShuffleBoardLayout();

    this.configureBindings();
    if (Constants.isCompRobot) {

      this.elevator.setDefaultCommand(new ElevatorCommand(() -> this.copilotController.getRawAxis(0), this.elevator));
      intake.setDefaultCommand(
          new SensorFeedCommand(intake, () -> this.copilotController.getHID().getRawButton(4), this.led));

      NamedCommands.registerCommand("IntakeNote", new SensorFeedCommand(this.intake, () -> true, this.led));
      NamedCommands.registerCommand("ShootyNote", this.intake.setStateCommand(IntakeState.INTAKE));
      NamedCommands.registerCommand("FeedInitalNote", this.intake.setStateCommand(IntakeState.FEED));
      NamedCommands.registerCommand("ShooterSpeaker", new InstantCommand(() -> this.shooter.setShooterSpeed(4500)));
      NamedCommands.registerCommand("ShooterPodium", new InstantCommand(() -> this.shooter.setShooterSpeed(5000)));
      NamedCommands.registerCommand("ShooterStop", new InstantCommand(() -> this.shooter.setShooterSpeed(0)));
      NamedCommands.registerCommand("ShooterUp", new InstantCommand(() -> this.shooter.setShooterAngle(false)));
      NamedCommands.registerCommand("ShooterDown", new InstantCommand(() -> this.shooter.setShooterAngle(true)));

      m_AutoChooser = AutoBuilder.buildAutoChooser("Four_Note_Long_Auto");
      SmartDashboard.putData(m_AutoChooser);
    }

    // m_AutoChooser.getSelected();

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      Logger.recordOutput("Pathing/curPose");
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      Logger.recordOutput("Pathing/targetPose");
    });


    // Logging callback for the active path, this is sent as a list of poses
    // PathPlannerLogging.setLogActivePathCallback((poses) -> {
    // // Do whatever you want with the poses here
    // field.getObject("path").setPoses(poses);
    // });

    this.led.setDefaultCommand(this.led.shooterIntakeFlash(this.shooter, this.intake));
  }

  private void configureBindings() {
    // Intake bindings
    if (Constants.isCompRobot) {

      // this.driveController.leftBumper().whileTrue(intake.setStateCommand(IntakeState.INTAKE));
      this.copilotController.button(5).whileTrue(intake.setStateCommand(IntakeState.EJECT));

      // hoodDeployBtn.onTrue(new InstantCommand(() ->
      // shooter.setHoodPosition(HoodPosition.DEPLOYED)))
      // .onFalse(new InstantCommand(() ->
      // shooter.setHoodPosition(HoodPosition.STOWED)));

      this.driveController.leftBumper().onTrue(new InstantCommand(() -> shooter.setShooterAngle(false)))
          .onFalse(new InstantCommand(() -> shooter.setShooterAngle(true)));

      climberBreaksBtn.onTrue(new InstantCommand(() -> this.elevator.setBreaks(true)))
          .onFalse(new InstantCommand(() -> this.elevator.setBreaks(false)));


      this.driveController.button(7).onTrue(new InstantCommand( () -> {
        this.imu.reset();
      }));
      // this.driveController.leftBumper().whileTrue(new RunCommand(() -> {
      // if (this.driveController.getHID().getRightBumper()) {

      // }

      // this.shooter.setShooterSpeed(this.driveController.getHID().getRightBumper() ?
      // 500 : 0);
      // this.shooter.setEERoller(this.driveController.getHID().getRightBumper() ? 0.7
      // : 0);
      // this.shooter.setHood(true);
      // }, this.shooter)
      // .finallyDo(() -> {
      // this.shooter.setShooterSpeed(0);
      // this.shooter.setEERoller(0);
      // this.shooter.setHood(false);
      // }));

      var feedCommand = new ParallelCommandGroup(
        this.intake.setStateCommand(IntakeState.INTAKE),
        new RunCommand(() -> {
          if (this.shooter.isHoodSetUp()) {
            this.shooter.setEERoller(0.8);
          } else {
            this.shooter.setEERoller(0);
          }
        }).finallyDo(() -> {
          this.shooter.setEERoller(0);
        // }));
        })).onlyIf(() -> this.shooter.isShooterAtSP() && this.shooter.isHoodAtSP() && this.shooter.getShooterSPRPM() > 0);

      var reverseFeedCommand = new RunCommand(() -> {
        if (this.shooter.isHoodAtSP() && this.shooter.isHoodSetUp() && this.hoodOverrideBtn.getAsBoolean()) {
          this.shooter.setEERoller(-0.8);
        }
        else {
          this.shooter.setEERoller(0);
        }
      }).finallyDo(() -> this.shooter.setEERoller(0));

      this.driveController.rightTrigger(0.5).whileTrue(feedCommand).whileFalse(reverseFeedCommand);

      this.driveController.leftTrigger(0.5).whileTrue(new RunCommand(() -> {
        this.shooter.setShooterSpeed(600);
        this.shooter.setHood(true);
      }).finallyDo(() -> {
        this.shooter.setShooterSpeed(0);
        this.shooter.setHood(false);
      }));
    }
  }

  public Command getAutonomousCommand() {
    // return new FollowPathHolonomic(
    // PathingConstants.basicPath,
    // this.driveTrain::getPose,
    // this.driveTrain::getSpeed,
    // this.driveTrain::drive,
    // PathingConstants.pathFollowerConfig,
    // () -> true,
    // this.driveTrain);
    // throw new UnsupportedOperationException();

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Two_Note");

    // return AutoBuilder.followPath(path);
    return m_AutoChooser.getSelected();
    
    // return new PathPlannerAuto());
  }

  public Command getTeleopCommand() {
    // return new TeleopSwerve(driveTrain, shooter, elevator, () ->
    // this.driveController.getLeftY(),
    // () -> this.driveController.getLeftX(),
    // () -> this.driveController.getRightX(),
    // this::copilotGetState,
    // () -> -this.copilotController.getX(),
    // () -> this.copilotController.getHID().getRawButton(7));

    // return new TeleopSwerve(this.driveTrain, this.driveController.getHID(), this.vision);
    return this.shooter.initHoodCommand()
      .andThen(new TeleopSwerve(this.driveTrain, this.driveController.getHID(), this.vision, this.shooter, this.copilotController));
    // return this.led.setLEDs(255, 255, 255);
    // return this.led.animate(new RainbowAnimation());
  }

  public Command getTestCommand() {
    return new TestCommand(this.driveTrain);
  }
}
