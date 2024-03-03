
package frc.robot.constants;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Various constants used throughout the program are defined here. They're
 * defined here instead of elsewhere just for ease of changing them.
 */
public final class Constants {
    public static final boolean simReplay = false;
    public static final double controllerDeadband = 0.1;
    public static final int pigeonId = 10;

    public static final int testSampleCount = 1024;
    public static final int testStepCount = 5;
    public static final double testMaxVoltage = 0.2;
    public static final double testMinVoltage = 0.05;
    public static final double testMaxDistance = 2;
    public static final String testMeasurementFName = "test_measurement.csv";
    public static final String testResidualsFName = "test_residuals.csv";

    public static final int driveControllerPort = 1;
    public static final int copilotControllerPort = 0;

    public static final int pcmId = 40;

    public static final int brakeSolenoidPort = 0;
    public static final int shooterSolenoidPort = 3;

    public static final boolean isCompRobot;

    public static final int[] driveBaseMac = { 0x00, 0x80, 0x2f, 0x17, 0x7d, 0x91 };

    // this annoys me so much for some reason
    private static boolean intByteEq(int[] a, byte[] b) {
        if (a.length != b.length) {
            return false;
        }
        for (int i = 0; i < a.length; i++) {
            if ((byte) a[i] != b[i]) {
                return false;
            }
        }
        return true;
    }

    private static boolean isCompRobot() throws SocketException, UnknownHostException {
        if (RobotBase.isSimulation()) {
            return true;
        }

        byte[] mac = null;
        mac = NetworkInterface.getByInetAddress(InetAddress.getByName("10.88.41.2")).getHardwareAddress();
        
        if (mac != null && intByteEq(driveBaseMac, mac)) {
            return false;
        }

        if (mac == null) {
            System.out.println("getHardwareAddress returned null! Defaulting to comp robot...");
        }

        return true;
    }

    static {
        boolean isCompRobotTmp;

        try {
            isCompRobotTmp = isCompRobot();
        }
        catch (SocketException | UnknownHostException e) {
            e.printStackTrace();
            System.out.println("Failed to get robot mac address! Defaulting to comp robot...");
            isCompRobotTmp = true;
        }

        isCompRobot = isCompRobotTmp;

        System.out.format("Detected robot as %s\n", isCompRobot ? "competition" : "drivebase");
    }

    public static final class CandleConstants {
        public static final int kCandleID = 22;

        public static final int kLEDCount = 150;

        public static final double kMaxBrightness = 0.7;
    }

    public static final class ElevatorConstants {
        public static final int kElevatorMain = 20;
        public static final int kElevatorFollower = 21;

        public static final int kElevatorBottomSensor = 3;
        public static final int kElevatorTopSensor = 4;

        public static final int kBreaksPort = 2;
    }

}
