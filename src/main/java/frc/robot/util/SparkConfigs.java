package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

public class SparkConfigs {
    
    public static void configureSparkMax(CANSparkMax spark, int currentLimit, IdleMode idleMode) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(currentLimit);
        spark.setIdleMode(idleMode);
    }


}
