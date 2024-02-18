package frc.lib.util;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Notifier;

// Four layers of inheritance??? AAAA

/**
 * `CANSparkMax` with motor saftey capabilities.
 * Should be prefered over just `CANSparkMax` in this codebase.
 */
public class SafeCANSparkMax extends CANSparkMax {
    private static class MotorSafetyImpl extends MotorSafety {
        private String description;
        private CANSparkMax controller;

        // Repeatedly call `stopMotor`. This is so you can't command the motor once it times out.
        private Notifier stopNotifier = new Notifier(() -> {
            synchronized (this.controller) {
                // it's fine to call `stopMotor` when the controller is closed looping because `stopMotor`, in 
                // `CANSparkMax`, and `setReference`, in `SparkPIDController`, both just call `setpointCommand`, in 
                // `CANSparkLowLevel`.
                this.controller.stopMotor();
            }
        });

        public MotorSafetyImpl(CANSparkMax controller, String description) {
            super();
            this.description = description;
            this.controller = controller;
        }

        @Override
        public void feed() {
            super.feed();
            this.stopNotifier.stop();
        }

        @Override
        public void stopMotor() {
            synchronized (this.controller) {
                // Don't wait for the HAL
                this.controller.stopMotor();
            }

            this.stopNotifier.startPeriodic(0.02);
        }

        @Override
        public String getDescription() {
            return this.description;
        }
    }

    private MotorSafetyImpl safety = new MotorSafetyImpl(this, "CANSparkMax" + this.getDeviceId());

    public SafeCANSparkMax(int deviceId, MotorType type, double expirationTime) {
        super(deviceId, type);
        this.safety.setSafetyEnabled(true);
        this.safety.setExpiration(expirationTime);
    }

    public SafeCANSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
        this.safety.setSafetyEnabled(true);
    }

    /**
     * Feeds the motor safety object.
     * Has to be called seperately from `set`, because we'd have to intercept the call to `setReference` in 
     * `SparkPIDController` too.
     */
    public void feed() {
        this.safety.feed();
    }

    /**
     * Determine if the motor is still operating or has timed out.
     * @return true if the motor is still operating normally and hasn't timed out.
     */
    public boolean isAlive() {
        return this.safety.isAlive() || (this.get() != 0); // Direct floating point comparison is on purpose
    }
}
