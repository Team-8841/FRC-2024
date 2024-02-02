package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.Supplier;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.sensors.imu.IMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class FFTestCommand extends Command {
    private DriveTrainSubsystem driveTrain;
    private IMU imu;
    private TestState state;
    private int curSample;
    private double beginDist, curVoltage = Constants.testMinVoltage, lastSpeed = -1, initialAngle,
            stepSize = (Constants.testMaxVoltage - Constants.testMinVoltage) / Constants.testStepCount;
    private double[] voltages = new double[Constants.testSampleCount];
    private double[][] measurements = new double[Constants.testSampleCount][2];
    private PIDController returnPID = new PIDController(0.1, 0, 0), yawPID = new PIDController(0.1, 0, 0);

    public static enum TestState {
        MEASURING,
        ALIGNING,
        RETURNING;
    };

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

    public FFTestCommand(DriveTrainSubsystem driveTrain, IMU imu) {
        this.imu = imu;

        this.driveTrain = driveTrain;
        driveTrain.setDrivePIDs(0, 1, 0, 0, 0, 0);
        this.beginDist = this.getDistance();

        this.yawPID.setTolerance(0.5);

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
    public void initialize() {
        this.state = TestState.MEASURING;
        this.initialAngle = this.imu.getAngle().getDegrees();
    }

    @Override
    public void execute() {
        Logger.recordOutput("TestSuite/FFTest/currentState", this.state.name());
        switch (this.state) {
        case MEASURING:
            this.driveTrain.drive(new Translation2d(curVoltage, 0), 0, false);
            double curSpeed = this.getSpeed(this.driveTrain.getSpeed());

            if (this.lastSpeed > 0) {
                measurements[this.curSample][0] = curSpeed;
                voltages[this.curSample] = curVoltage;
                measurements[this.curSample][1] = (curSpeed - lastSpeed) / 0.02;

                Logger.recordOutput("TestSuite/FFTest/voltage", voltages[curSample]);
                Logger.recordOutput("TestSuite/FFTest/omega", measurements[curSample][0]);
                Logger.recordOutput("TestSuite/FFTest/omegaDot", measurements[curSample][1]);
                Logger.recordOutput("TestSuite/FFTest/curSample", curSample);

                curSample += 1;
            }

            this.lastSpeed = curSpeed;

            if (this.getDistance() >= Constants.testMaxDistance) {
                // We've finished testing for this voltage, drive back
                this.driveTrain.drive(new Translation2d(), 0, true);
                this.state = TestState.ALIGNING;
                System.out.println("[TestSuite] Stopping measuring, aligning...");
            }
            break;
        case ALIGNING:
            this.driveTrain.resetDrivePIDs();
            double ctrlEffort = this.yawPID.calculate(this.imu.getAngle().getDegrees(), this.initialAngle);
            this.driveTrain.drive(new Translation2d(), ctrlEffort, true);
            if (this.yawPID.atSetpoint()) {
                this.driveTrain.setDrivePIDs(0, 1, 0, 0, 0, 0);
                this.state = TestState.RETURNING;
                System.out.println("[TestSuite] Stopping aligning, driving back...");
            }
            break;
        case RETURNING:
            var speed = Math.max(
                    Math.min(this.returnPID.calculate(this.getDistance()),
                            Constants.testMaxVoltage),
                    -Constants.testMaxVoltage);
            this.driveTrain.drive(new Translation2d(speed, 0), 0, true);
            Logger.recordOutput("/TestSuite/FFTest/ReturnError", this.returnPID.getPositionError());
            if (Math.abs(this.returnPID.getPositionError()) < 0.15) {
                // We're back at the start, start measuring with a new voltage
                this.driveTrain.drive(new Translation2d(), 0, true);
                this.lastSpeed = -1;
                this.curVoltage = this.curVoltage + this.stepSize <= Constants.testMaxVoltage
                        ? this.curVoltage + this.stepSize
                        : Constants.testMinVoltage;

                this.state = TestState.MEASURING;
                System.out.println("[TestSuite] Stopping driving back, measuring...");
                System.out.println("[TestSuite] Resuming measuring with a voltage of " + this.curVoltage);
            }
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.drive(new Translation2d(0, 0), 0, false);
        this.driveTrain.resetDrivePIDs();
        this.driveTrain.resetSteeringPIDs();

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