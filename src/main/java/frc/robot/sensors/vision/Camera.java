package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    public String name;

    public Camera(String name) {
        this.name = name;
    }
}
