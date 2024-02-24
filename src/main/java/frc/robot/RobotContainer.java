// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SensorFeedCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.PathingConstants;
import frc.robot.constants.swerve.PureTalonFXConstants;
import frc.robot.sensors.imu.DummyIMU;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.NavX2;
import frc.robot.sensors.imu.SimIMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.drive.DummySwerveModuleIO;
import frc.robot.subsystems.drive.SimSwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.TalonFXSwerveModuleIO;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.RealShooterIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.intake.DummyIntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem driveTrain;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;

  // Controllers
  private CommandXboxController driveController;
  private CommandJoystick copilotController;

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
      this.shooter = new ShooterSubsystem(new RealShooterIO());
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

    this.driveController = new CommandXboxController(1);
    this.copilotController = new CommandJoystick(0);
    this.configureBindings(this.driveController, this.copilotController);
  }

  private void configureBindings(CommandXboxController controller, CommandJoystick copilot) {
    controller.rightTrigger(0.5).whileTrue(this.intake.setStateCommand(IntakeState.INTAKE));
    // copilot.button(4).whileTrue(new SensorFeedCommand(this.intake));
    //copilot.button(4).whileTrue(this.intake.setStateCommand(Intakes));
    copilot.button(5).whileTrue(this.intake.setStateCommand(IntakeState.EJECT));

    this.intake.setDefaultCommand(new SensorFeedCommand(this.intake, () -> copilot.getHID().getRawButton(4)));
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

  private ShooterState copilotGetState() {
    double potVals[] = {1, 0.55, 0.06, -0.44, -0.96};
    ShooterState potStates[] = {ShooterState.OFF, ShooterState.COOLAMP, ShooterState.COOLSHOT,ShooterState.COOLSHOT, ShooterState.COOLSHOT};
    double epsilon = 0.1;
    double pot = this.copilotController.getRawAxis(3);

    for (int i = 0; i < potVals.length; i++) {
      if (potVals[i] - epsilon <= pot && potVals[i] + epsilon >= pot) {
        return potStates[i];
      }
    }

    return ShooterState.OFF;
  }

  public Command getTeleopCommand() {
    // return new TeleopSwerve(driveTrain, () -> -this.driveController.getLeftY(),
    //     () -> -this.driveController.getLeftX(),
    //     () -> -this.driveController.getRightX());
    return new TeleopSwerve(driveTrain, shooter, () -> this.driveController.getLeftY(),
        () -> this.driveController.getLeftX(),
        () -> this.driveController.getRightX(),
        this::copilotGetState);
  }

  public Command getTestCommand() {
    return new TestCommand(this.driveTrain);
  }
}
