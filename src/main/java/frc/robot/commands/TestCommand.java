package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class TestCommand extends Command {
  DriveTrainSubsystem driveTrain;

  public TestCommand(DriveTrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
  }

  @Override
  public void execute() {
    // Spin wheels at 1m/s forward
    driveTrain.drive(new Translation2d(1, 0), 0, false);
  }
}
