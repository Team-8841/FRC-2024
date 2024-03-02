// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem driveTrain;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ElevatorSubsystem elevator;

  private final LEDSubsystem led = new LEDSubsystem();

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

    private double climberMovement;

    private double getShooterState() {

        double potVals[] = { 1, 0.55, 0.06, -0.44, -0.96 };
        double potRPM[] = { 0, 500, 1000, 2000, 3000 };
        double epsilon = 0.1;
        double pot = this.copilotController.getRawAxis(3);

        for (int i = 0; i < potVals.length; i++) {
            if (potVals[i] - epsilon <= pot && potVals[i] + epsilon >= pot) {
                return potRPM[i];
            }
        }

        return 0;
    }



  public RobotContainer(CommandXboxController driveController, CommandJoystick copilotController) {
    this.driveController = driveController;
    this.copilotController = copilotController;

      compressorOverrideBtn = new JoystickButton(copilotController.getHID(), OIConstants.button1);
      hoodDeployBtn = new JoystickButton(        copilotController.getHID(), OIConstants.button2);
      hoodOverrideBtn = new JoystickButton(      copilotController.getHID(), OIConstants.button3);
      intakeInBtn = new JoystickButton(          copilotController.getHID(), OIConstants.button4);
      intakeOutBtn = new JoystickButton(         copilotController.getHID(), OIConstants.button5);
      shooterAngleBtn = new JoystickButton(      copilotController.getHID(), OIConstants.button6);    
      climberBreaksBtn = new JoystickButton(     copilotController.getHID(), OIConstants.button7);
      climberMovement = copilotController.getHID().getRawAxis(OIConstants.analog1);

      SwerveModuleIO swerveModules[];
        if (RobotBase.isReal()) {
      // Real robot
      if (Constants.isCompRobot) {
        swerveModules = new SwerveModuleIO[] {
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod0.constants, false),
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod1.constants, false),
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod2.constants, false),
            new TalonFXSwerveModuleIO(CompRobotConstants.Mod3.constants, false),
            // new DummySwerveModuleIO(),
        };
      }
      else {
        swerveModules = new SwerveModuleIO[] {
            //new DummySwerveModuleIO(),
            //new DummySwerveModuleIO(),
            new MixedSwerveModuleIO(BaseRobotConstants.Mod1.constants, false),
            new MixedSwerveModuleIO(BaseRobotConstants.Mod0.constants, false),
            new MixedSwerveModuleIO(BaseRobotConstants.Mod3.constants, false),
            new MixedSwerveModuleIO(BaseRobotConstants.Mod2.constants, false),
            //new DummySwerveModuleIO(),
            // new DummySwerveModuleIO(),
            // new DummySwerveModuleIO(),
            // new DummySwerveModuleIO(),
        };

      }

      // this.imu = new Pigeon2IO(Constants.pigeonId);
      this.imu = new NavX2();

      this.intake = new IntakeSubsystem(Constants.isCompRobot ? new RealIntakeIO() : new DummyIntakeIO());
      this.shooter = new ShooterSubsystem();
      //this.elevator = new ElevatorSubsystem(Constants.isCompRobot ? new RealElevatorIO() : new DummyElevatorIO());
      this.elevator = new ElevatorSubsystem();
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

    this.shooter.setDefaultCommand(new RunCommand(() -> {
        this.shooter.setShooterSpeed(getShooterState());

        if(hoodDeployBtn.getAsBoolean()) {
            this.shooter.setEEAngle(64);
        } else {
            this.shooter.setEEAngle(0);
        }

        
    }, this.shooter));
  }

  private void configureBindings() {
    // Intake bindings
    if (this.copilotController != null) {
      //this.driveController.leftBumper().whileTrue(intake.setStateCommand(IntakeState.INTAKE));
      this.copilotController.button(5).whileTrue(intake.setStateCommand(IntakeState.EJECT));
      intake.setDefaultCommand(new SensorFeedCommand(intake, () -> this.copilotController.getHID().getRawButton(4)));
      shooterAngleBtn.onTrue(new InstantCommand(() -> shooter.setShooterAngle(true)))
                      .onFalse(new InstantCommand(() -> shooter.setShooterAngle(false)));
      this.elevator.setDefaultCommand(new ElevatorCommand(() -> this.copilotController.getRawAxis(0), this.elevator));             
        climberBreaksBtn.onTrue(new InstantCommand(() ->  this.elevator.setBreaks(true)))
                        .onFalse(new InstantCommand(() -> this.elevator.setBreaks(false)));
      this.driveController.leftBumper().whileTrue(new RunCommand(() -> {
        //if (this.shooter.isShooterAtSpeed() && this.shooter.getShooterSpeed() < 400) {
        //  this.shooter.setEERoller(0.1);
        //  this.intake.setIntakeState(IntakeState.EJECT);
        //} else {
        //  this.shooter.setEERoller(0);
        //  this.intake.setIntakeState(IntakeState.OFF);
        //}
        Logger.recordOutput("shooterRoller", Logger.getRealTimestamp());
        this.shooter.setEERoller(0.1);
    }).finallyDo(() -> this.shooter.setEERoller(0)));
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

    return new TeleopSwerve(this.driveTrain, this.driveController.getHID(),
        this.copilotController == null ? null : this.copilotController.getHID());
  }

  public Command getTestCommand() {
    return new TestCommand(this.driveTrain);
  }
}
