package frc.robot.util;

public class Gains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kV;
    
    public Gains(double _kP, double _kI, double _kD, double _kV){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kV = _kV;
    }
}
