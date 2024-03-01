// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SensorFeedCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.CompRobotConstants;
import frc.robot.sensors.imu.DummyIMU;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.NavX2;
import frc.robot.sensors.imu.SimIMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.drive.DummySwerveModuleIO;
import frc.robot.subsystems.drive.SimSwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.TalonFXSwerveModuleIO;
import frc.robot.subsystems.elevator.DummyElevatorIO;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RealElevatorIO;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.DummyShooterIO;
import frc.robot.subsystems.shooter.RealShooterIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.DummyIntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem driveTrain;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ElevatorSubsystem elevator;

  // Controllers
  private CommandXboxController driveController;
  private CommandJoystick copilotController;

  public RobotContainer(CommandXboxController driveController, CommandJoystick copilotController) {
    this.driveController = driveController;
    this.copilotController = copilotController;

    SwerveModuleIO swerveModules[];

    if (RobotBase.isReal()) {
      // Real robot
      swerveModules = new SwerveModuleIO[] {
          new TalonFXSwerveModuleIO(CompRobotConstants.Mod0.constants, false),
          new TalonFXSwerveModuleIO(CompRobotConstants.Mod1.constants, false),
          new TalonFXSwerveModuleIO(CompRobotConstants.Mod2.constants, false),
          new TalonFXSwerveModuleIO(CompRobotConstants.Mod3.constants, false),
          // new DummySwerveModuleIO(),
          // new DummySwerveModuleIO(),
          // new DummySwerveModuleIO(),
          // new DummySwerveModuleIO(),
      };

      // this.imu = new Pigeon2IO(Constants.pigeonId);
      this.imu = new NavX2();

      this.intake = new IntakeSubsystem(Constants.isCompRobot ? new RealIntakeIO() : new DummyIntakeIO());
      this.shooter = new ShooterSubsystem(Constants.isCompRobot ? new RealShooterIO() : new DummyShooterIO());
      this.elevator = new ElevatorSubsystem(Constants.isCompRobot ? new RealElevatorIO() : new DummyElevatorIO());
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

    ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");
    this.imu.initializeShuffleBoardLayout(robotTab.getLayout("IMU", BuiltInLayouts.kList));

    this.configureBindings();
  }

  private void configureBindings() {
    // Intake bindings
    this.driveController.rightTrigger(0.5).whileTrue(intake.setStateCommand(IntakeState.INTAKE));
    this.copilotController.button(5).whileTrue(intake.setStateCommand(IntakeState.EJECT));
    intake.setDefaultCommand(new SensorFeedCommand(intake, () -> this.copilotController.getHID().getRawButton(4)));
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
    throw new UnsupportedOperationException();
  }

  public Command getTeleopCommand() {
    // return new TeleopSwerve(driveTrain, shooter, elevator, () ->
    // this.driveController.getLeftY(),
    // () -> this.driveController.getLeftX(),
    // () -> this.driveController.getRightX(),
    // this::copilotGetState,
    // () -> -this.copilotController.getX(),
    // () -> this.copilotController.getHID().getRawButton(7));

    return new TeleopSwerve(this.driveTrain, this.shooter, this.elevator, this.driveController.getHID(),
        this.copilotController.getHID());
  }

  public Command getTestCommand() {
    return new TestCommand(this.driveTrain);
  }
}
