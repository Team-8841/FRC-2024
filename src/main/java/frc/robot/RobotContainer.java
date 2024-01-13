// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.PathingConstants;
import frc.robot.constants.swerve.PureTalonFXConstants;
import frc.robot.sensors.imu.DummyIMU;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.Pigeon2IO;
import frc.robot.sensors.imu.SimIMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.drive.DummySwerveModuleIO;
import frc.robot.subsystems.drive.SimSwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.TalonFXSwerveModuleIO;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem driveTrain;

  // Controllers
  private CommandXboxController driveController;

  public RobotContainer() {
    SwerveModuleIO swerveModules[];

    if (RobotBase.isReal()) {
      // Real robot
      swerveModules = new SwerveModuleIO[] {
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod0.constants),
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod1.constants),
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod2.constants),
          new TalonFXSwerveModuleIO(PureTalonFXConstants.Mod3.constants),
      };

      this.imu = new Pigeon2IO(Constants.pigeonId);
    } else if (Constants.simReplay) {
      // Replay
      swerveModules = new SwerveModuleIO[] {
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
          new DummySwerveModuleIO(),
      };

      this.imu = new DummyIMU();
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

  private void configureBindings(CommandXboxController controller) {
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
    return new TestCommand(driveTrain);
  }
}
