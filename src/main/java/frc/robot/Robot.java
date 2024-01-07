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
  private Command autonomousCommand, teleopCommand;
  private RobotContainer robotContainer;
  private Logger logger = Logger.getInstance();

  @SuppressWarnings("unused")
  private PowerDistribution pdh;

  @Override
  public void robotInit() {
    
    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    if (isReal()) {
      // Log to usb stick
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); 
      // Logs to NT4
      logger.addDataReceiver(new NT4Publisher()); 
      // Enables logging of PDH data
      //this.pdh = new PowerDistribution(1, ModuleType.kRev); 
    } else if (Constants.simReplay) {
      // Run as fast as possible
      setUseTiming(false); 
      // Get the replay log from AdvantageScope (or prompt the user)
      String logPath = LogFileUtil.findReplayLog();
      // Read replay log
      logger.setReplaySource(new WPILOGReader(logPath)); 
      // Log to a file
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    } else {
      // Log to a file
      logger.addDataReceiver(new WPILOGWriter("/tmp/sim.wpilog"));
      // Logs to NT4
      logger.addDataReceiver(new NT4Publisher()); 
    }

    // Starts advantagekit's logger
    logger.start();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    var canStatus = RobotController.getCANStatus();
    logger.recordOutput("/CANStatus/busOffCount", canStatus.busOffCount);
    logger.recordOutput("/CANStatus/percentBusUtilization", canStatus.percentBusUtilization);
    logger.recordOutput("/CANStatus/receiveErrorCount", canStatus.receiveErrorCount);
    logger.recordOutput("/CANStatus/transmitErrorCount", canStatus.transmitErrorCount);
    logger.recordOutput("/CANStatus/txFullCount", canStatus.txFullCount);

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
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
