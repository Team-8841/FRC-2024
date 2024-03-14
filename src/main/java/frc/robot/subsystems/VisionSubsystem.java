package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-intake");

    // gets
    private final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private final NetworkTableEntry ty = limelightTable.getEntry("ty");
    private final NetworkTableEntry ta = limelightTable.getEntry("ta");
    private final NetworkTableEntry tc = limelightTable.getEntry("tc");

    // sets
    //private final NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");
    //private final NetworkTableEntry camMode = limelightTable.getEntry("camMode");
    // private final NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
    // private final NetworkTableEntry stream = limelightTable.getEntry("stream");
    // private final NetworkTableEntry snapshot = limelightTable.getEntry("snapshot");

    public VisionSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateStatus();
    }

    public double isTargetValid() {
        return tv.getDouble(0.0);
    }

    public double getTargetHorizontalOffset() {
        if (isTargetValid() > 0.5) {
            return tx.getDouble(0.0);
        } else {
            return 0;
        }
    }

    public double getTargetVerticalOffset() {
        if (isTargetValid() > 0.5) {
            return ty.getDouble(0.0);
        } else {
            return 0;
        }
    }

    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public double getTravelSpeed(){
        double headingError = getTargetHorizontalOffset();
        double driveSpeed = VisionConstants.kAssistkP * headingError;

        double minSpeed = VisionConstants.kMinAssistSpeed;
        double threshold = 0.45;

        if (Math.abs(driveSpeed) < minSpeed && Math.abs(headingError) > threshold) {
            if (driveSpeed< 0) {
              driveSpeed = -minSpeed;
            } else {
              driveSpeed = minSpeed;
            }
        }

        return driveSpeed;
    }

    public double getDistance() {
        // TODO: update values
        double heightOfCamera = 43;
        double heightOfTarget = 29;
        double angleOfCamera = -20;
        double angleofTarget = getTargetVerticalOffset();
        return (heightOfTarget - heightOfCamera)
                / Math.tan(Math.toRadians(angleOfCamera + angleofTarget));
    }

    public void updateStatus() {
        SmartDashboard.putNumber("[Vision] Valid Target", isTargetValid());
        SmartDashboard.putNumber("[Vision] Target Horizontal Offset", tx.getDouble(0.0));
        SmartDashboard.putNumber("[Vision] Target Vertical Offset", ty.getDouble(0.0));
        SmartDashboard.putNumber("[Vision] Target Area", getTargetArea());
        //SmartDashboard.putNumber("[Vision] Distance", getDistance());

        Logger.recordOutput("Vision/validTarget", this.isTargetValid());
        Logger.recordOutput("Vision/horizOffset", this.tx.getDouble(0));
        Logger.recordOutput("Vision/vertOffset", this.ty.getDouble(0));
        Logger.recordOutput("Vision/area", this.getTargetArea());
    }
}
