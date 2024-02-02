package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.Supplier;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class TestCommand extends Command {
    private DriveTrainSubsystem driveTrain;
    private boolean isMeasuring = true;
    private int curSample;
    private double beginDist, curVoltage = Constants.testMinVoltage, lastSpeed = -1, stepSize = (Constants.testMaxVoltage - Constants.testMinVoltage) / Constants.testStepCount;
    private double[] voltages = new double[Constants.testSampleCount];
    private double[][] measurements = new double[Constants.testSampleCount][2];
    private PIDController returnPID = new PIDController(0.1, 0, 0);

    private Supplier<Thread> dumpMeasurementsThread = () -> new Thread(() -> {
            try {
                var outPath = new File(Filesystem.getOperatingDirectory(), Constants.testMeasurementFName);
                System.out.format("[TestSuite] Dumping measurements to `%s`\n", outPath.getAbsolutePath());
                var writer = new FileWriter(outPath);
                writer.write("voltage,omega,omegaDot\n");
                for (int i = 0; i < Constants.testSampleCount; i++) {
                    var line = String.format("%f,%f,%f\n", voltages[i], measurements[i][0], measurements[i][1]);
                    writer.write(line);
                }
                writer.flush();
                writer.close();
                System.out.println("[TestSuite] Done dumping measurements");
            } catch (IOException e) {
                System.out.println("[TestSuite] Failed to write measurements: " + e.toString());
                e.printStackTrace();
            }
        });

    private Supplier<Thread> calculationThread = () -> new Thread(() -> {
            var regression = new OLSMultipleLinearRegression();
            regression.newSampleData(voltages, measurements);
            var estimatedParams = regression.estimateRegressionParameters();

            String paramString = "{ ";
            for (var param : estimatedParams)
                paramString += param + ", ";
            paramString += "}";

            System.out.format("[TestSuite] params = %s\n", paramString);
            System.out.format("[TestSuite] kS = %f kV = %f kA = %f\n", estimatedParams[0], estimatedParams[1],
                    estimatedParams[2]);
            System.out.format("[TestSuite] r^2 = %f std. error = %f\n", regression.calculateRSquared(),
                    regression.estimateRegressionStandardError());

            try {
                var outPath = new File(Filesystem.getOperatingDirectory(), Constants.testResidualsFName);
                System.out.format("[TestSuite] Dumping residuals to `%s`\n", outPath.getAbsolutePath());
                var writer = new FileWriter(outPath);
                writer.write("voltage,omega,omegaDot,residual\n");
                var residuals = regression.estimateResiduals();
                for (int i = 0; i < Constants.testSampleCount; i++) {
                    var line = String.format("%f,%f,%f,%f\n", voltages[i], measurements[i][0], measurements[i][1],
                            residuals[i]);
                    writer.write(line);
                }
                writer.flush();
                writer.close();
                System.out.println("[TestSuite] Done dumping residuals");
            } catch (IOException e) {
                System.out.println("[TestSuite] Failed to write residuals: " + e.toString());
                e.printStackTrace();
            }
        });

    public TestCommand(DriveTrainSubsystem driveTrain) {
        this.driveTrain = driveTrain;
        driveTrain.setDrivePIDs(0, 1, 0, 0, 0, 0);
        this.beginDist = this.getDistance();

        this.addRequirements(driveTrain);
    }

    private double getSpeed(ChassisSpeeds speeds) {
        return Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
    }

    private double getDistance() {
        return this.driveTrain.getModulePositions()[0].distanceMeters - this.beginDist;
    }

    @Override
    public void execute() {
        if (this.isMeasuring) {
            this.driveTrain.drive(new Translation2d(curVoltage, 0), 0, false);
            double curSpeed = this.getSpeed(this.driveTrain.getSpeed());

            if (this.lastSpeed > 0) {
                measurements[this.curSample][0] = curSpeed;
                voltages[this.curSample] = curVoltage;
                measurements[this.curSample][1] = (curSpeed - lastSpeed) / 0.02;

                Logger.recordOutput("TestSuite/voltage", voltages[curSample]);
                Logger.recordOutput("TestSuite/omega", measurements[curSample][0]);
                Logger.recordOutput("TestSuite/omegaDot", measurements[curSample][1]);
                Logger.recordOutput("TestSuite/curSample", curSample);

                curSample += 1;
            }

            this.lastSpeed = curSpeed;

            if (this.getDistance() >= Constants.testMaxDistance) {
                // We've finished testing for this voltage, drive back
                this.driveTrain.drive(new Translation2d(), 0, false);
                this.isMeasuring = false;
                System.out.println("[TestSuite] Stopping measuring, driving back...");
            }
        } else {
            var speed = Math.max(
                    Math.min(this.returnPID.calculate(this.getDistance()),
                            Constants.testMaxVoltage),
                    -Constants.testMaxVoltage);
            this.driveTrain.drive(new Translation2d(speed, 0), 0, false);
            Logger.recordOutput("/TestSuite/ReturnError", this.returnPID.getPositionError());
            if (Math.abs(this.returnPID.getPositionError()) < 0.15) {
                // We're back at the start, start measuring with a new voltage
                this.driveTrain.drive(new Translation2d(), 0, false);
                this.lastSpeed = -1;
                this.curVoltage = this.curVoltage + this.stepSize <= Constants.testMaxVoltage
                        ? this.curVoltage + this.stepSize
                        : Constants.testMinVoltage;
                this.isMeasuring = true;

                System.out.println("[TestSuite] Resuming measuring with a voltage of " + this.curVoltage);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.drive(new Translation2d(0, 0), 0, false);
        System.out.println("[TestSuite] Done measuring, going to dump and process data...");
        this.dumpMeasurementsThread.get().start();
        this.calculationThread.get().start();
    }

    @Override
    public boolean isFinished() {
        if (this.curSample < Constants.testSampleCount)
            return false;
        return true;
    }
}