// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.NoteAlign;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.PathingConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.PureTalonFXConstants;
import frc.robot.sensors.imu.DummyIMU;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.NavX2;
import frc.robot.sensors.imu.SimIMU;
import frc.robot.sensors.vision.Camera;
import frc.robot.sensors.vision.piecetracking.LimelightTracker;
import frc.robot.sensors.vision.poseestimation.LimelightEstimatorModuleIO;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.drive.DummySwerveModuleIO;
import frc.robot.subsystems.drive.SimSwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.TalonFXSwerveModuleIO;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.intake.DummyIntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem driveTrain;
  private IntakeSubsystem intake;

  // Controllers
  private CommandJoystick coolBoardThing;
  private CommandXboxController driveController;
  private double noteStrafe = 0;

  // Vision
  private Camera camera = new Camera("Intake");
  private LimelightTracker trackerCommand = new LimelightTracker(this.camera, VisionConstants.intakeCam);
  private NoteAlign noteAlignCommand = new NoteAlign(trackerCommand, (strafe) -> this.noteStrafe = strafe);

  public RobotContainer() {
    SwerveModuleIO swerveModules[];

    if (RobotBase.isReal()) {
      // Real robot
      swerveModules = new SwerveModuleIO[] {
          //new DummySwerveModuleIO(),
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod0.constants, false),
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod1.constants, false),
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod2.constants, false),
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod3.constants, false),
          //new DummySwerveModuleIO(),
          //new DummySwerveModuleIO(),
      };

      //this.imu = new Pigeon2IO(Constants.pigeonId);
      this.imu = new NavX2();
      this.intake = new IntakeSubsystem(new RealIntakeIO());
    } else if (Constants.simReplay) {
      // Replay
      swerveModules = new SwerveModuleIO[] {
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
      };

      this.imu = new DummyIMU();
      this.intake = new IntakeSubsystem(new DummyIntakeIO());
    }
    else {
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

    ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");
    this.imu.initializeShuffleBoardLayout(robotTab.getLayout("IMU", BuiltInLayouts.kList));

    this.driveController = new CommandXboxController(OIConstants.gamepadPort);
    this.coolBoardThing = new CommandJoystick(OIConstants.copilotPort);
    
    this.configureBindings(this.driveController, this.coolBoardThing);
  }

  private void configureBindings(CommandXboxController xboxController, CommandJoystick coolBoardThing) {
    var hid = coolBoardThing.getHID();
    new JoystickButton(hid, OIConstants.button2).whileTrue(this.intake.sensorFeedCommand());
    new JoystickButton(hid, OIConstants.button3).whileTrue(this.intake.setStateCommand(IntakeState.INTAKE));

    xboxController.leftBumper().whileTrue(this.noteAlignCommand);
  }

  public Command getAutonomousCommand() {
    return new FollowPathHolonomic(
      PathingConstants.basicPath, 
      this.driveTrain::getPose,
      this.driveTrain::getSpeed,
      this.driveTrain::drive,
      PathingConstants.pathFollowerConfig, 
      () -> true,
      this.driveTrain
    );
  }

  public Command getTeleopCommand() {
    return new TeleopSwerve(driveTrain, () -> -this.driveController.getLeftY(),
        () -> this.noteStrafe - this.driveController.getLeftX(),
        () -> -this.driveController.getRightX());
  }

  public Command getTestCommand() {
    return new TestCommand(this.driveTrain);
  }
}
