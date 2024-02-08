// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.sim.DeviceType;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.math.Conversions;
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
      this.intake = new IntakeSubsystem(new RealIntakeIO());
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

    this.driveController = new CommandXboxController(0);
    this.configureBindings(this.driveController);
  }

  //CANSparkMax feeder = new CANSparkMax(15, MotorType.kBrushless);
  //CANSparkMax feeder = new CANSparkMax(15, MotorType.kBrushless);

  private void configureBindings(CommandXboxController controller) {
    double rps = 4500 / 60;
    controller.x()
      .onTrue(new InstantCommand(() -> this.shooter.setShooterSetPoint(rps)))
      .onFalse(new InstantCommand(() -> this.shooter.setShooterSetPoint(0)));
    //controller.y()
    //  .onTrue(new InstantCommand(() -> this.feeder.set(0.5)))
    //  .onFalse(new InstantCommand(() -> this.feeder.set(0)));
    controller.y()
      .whileTrue(this.intake.setStateCommand(IntakeState.INTAKE));
    controller.a()
      .whileTrue(this.intake.setStateCommand(IntakeState.OUTAKE));
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
        () -> -this.driveController.getLeftX(),
        () -> -this.driveController.getRightX());
  }

  public Command getTestCommand() {
    return new TestCommand(this.driveTrain);
  }
}
