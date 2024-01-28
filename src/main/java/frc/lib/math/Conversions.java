package frc.lib.math;

public class Conversions {
    /**
     * @param meters Meters (or meters per second)
     * @param circumference Circumference of wheel
     * @param gearRatio Gear ratio between sensor and wheel
     * @return Rotations (or rotations per second)
     */
    public static double metersToRots(double meters, double circumference, double gearRatio) {
        return meters * gearRatio / circumference;
    }
    
    /**
     * @param rotations Rotations (or rotations per second)
     * @param circumference Circumference of wheel
     * @param gearRatio Gear Ratio between sensor and wheel
     * @return Meters (or meters per second)
     */
    public static double rotsToMeters(double rotations, double circumference, double gearRatio) {
        return rotations * circumference / gearRatio;
    }
}