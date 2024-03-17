package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants.CandleConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle;

    public LEDSubsystem(int id) {
        this.candle = new CANdle(id);
        this.candle.configLEDType(LEDStripType.RGB);
        this.candle.configBrightnessScalar(1);
        this.candle.configLOSBehavior(false);
    }

    private Command animate(Animation animation, double time) {
        animation.setNumLed(CandleConstants.kLEDCount);
        var animCommand = new RunCommand(() -> this.candle.animate(animation, 0), this)
                .finallyDo(() -> this.candle.clearAnimation(0));
        return time < 0 ? animCommand : new ParallelRaceGroup(animCommand, new WaitCommand(time));
    }

    private Command animate(Animation animation) {
        return this.animate(animation, -1);
    }

    private Command setLEDs(int r, int g, int b, double time) {
        var animCommand = new RunCommand(() -> this.candle.setLEDs(r, g, b), this)
                .finallyDo(() -> this.candle.setLEDs(0, 0, 0));
        return time < 0 ? animCommand : new ParallelRaceGroup(animCommand, new WaitCommand(time));
    }

    private Command setLEDs(int r, int g, int b) {
        return this.setLEDs(r, g, b, -1);
    }

    public Command shooterIntakeFlash(ShooterSubsystem shooter, IntakeSubsystem intake) {
        return new RunCommand(() -> {
            if (DriverStation.isDisabled()) {
                this.candle.animate(new RainbowAnimation(0.5, 0.7, CandleConstants.kLEDCount));
            }
            else if (shooter.isShooterAtSP() && shooter.getShooterSPRPM() >= 800) {
                // Flash green if shooter is at setpoint
                this.candle.animate(new SingleFadeAnimation(0, 0xff, 0, 0, 1, CandleConstants.kLEDCount));
            }
            else if (intake.getIndexSensor()) {
                // Flash blue if intake has a note
                this.candle.animate(new SingleFadeAnimation(0x33, 0x66, 0xff, 0, 1, CandleConstants.kLEDCount));
            }
            else {
                this.candle.animate(new SingleFadeAnimation(0, 0, 0, 0, 1, CandleConstants.kLEDCount));
            } 
        }, this).finallyDo(() -> new SingleFadeAnimation(0, 0, 0, 0, 1, CandleConstants.kLEDCount)).ignoringDisable(true);
    }
}
