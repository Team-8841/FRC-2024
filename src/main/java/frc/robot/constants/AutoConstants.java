package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class AutoConstants {
    public static final double MaxSpeedMetersPerSecond = 3;
    public static final double MaxAccelerationMetersPerSecondSquared = 3;
    public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double PXController = 1;
    public static final double PYController = 1;
    public static final double PThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);
}
