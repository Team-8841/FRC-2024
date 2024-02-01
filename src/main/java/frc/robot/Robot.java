// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;

public class Robot extends LoggedRobot {
  private Command autonomousCommand, teleopCommand, testCommand;
  private RobotContainer robotContainer;

  @SuppressWarnings("unused")
  private PowerDistribution pdh;

  @Override
  public void robotInit() {
    
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    if (isReal()) {
      // Log to usb stick
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); 
      // Logs to NT4
      Logger.addDataReceiver(new NT4Publisher()); 
      // Enables logging of PDH data
      //this.pdh = new PowerDistribution(22, ModuleType.kRev); 
    } else if (Constants.simReplay) {
      // Run as fast as possible
      this.setUseTiming(false); 
      // Get the replay log from AdvantageScope (or prompt the user)
      String logPath = LogFileUtil.findReplayLog();
      // Read replay log
      Logger.setReplaySource(new WPILOGReader(logPath)); 
      // Log to a file
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    } else {
      this.setUseTiming(true); 
      // Log to a file
      Logger.addDataReceiver(new WPILOGWriter("/tmp/sim.wpilog"));
      // Logs to NT4
      Logger.addDataReceiver(new NT4Publisher()); 
    }

    // Starts advantagekit's Logger
    Logger.start();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("CANStatus/busOffCount", canStatus.busOffCount);
    Logger.recordOutput("CANStatus/percentBusUtilization", canStatus.percentBusUtilization);
    Logger.recordOutput("CANStatus/receiveErrorCount", canStatus.receiveErrorCount);
    Logger.recordOutput("CANStatus/transmitErrorCount", canStatus.transmitErrorCount);
    Logger.recordOutput("CANStatus/txFullCount", canStatus.txFullCount);

    if (isSimulation()) {
      SimManager.getInstance().periodic();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    this.autonomousCommand = robotContainer.getAutonomousCommand();
    if (this.autonomousCommand != null) {
      this.autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    this.teleopCommand = this.robotContainer.getTeleopCommand();
    if (this.teleopCommand != null) {
      this.teleopCommand.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    this.testCommand = this.robotContainer.getTestCommand();
    if (this.testCommand != null) {
      this.testCommand.schedule();
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
