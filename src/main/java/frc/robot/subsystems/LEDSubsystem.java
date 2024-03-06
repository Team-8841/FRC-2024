package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants.CandleConstants;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle;

    public LEDSubsystem() {
        this.candle.configAllSettings(CandleConstants.config);
    }

    public Command animate(Animation animation, double time) {
        var animCommand = new InstantCommand(() -> this.candle.animate(animation, 0), this)
                .finallyDo(() -> this.candle.clearAnimation(0));
        return time < 0 ? animCommand : new ParallelRaceGroup(animCommand, new WaitCommand(time));
    }

    public Command animate(Animation animation) {
        return this.animate(animation, -1);
    }

    public Command setLEDs(int r, int g, int b, double time) {
        var animCommand = new InstantCommand(() -> this.candle.setLEDs(r, g, b), this)
                .finallyDo(() -> this.candle.setLEDs(0, 0, 0));
        return time < 0 ? animCommand : new ParallelRaceGroup(animCommand, new WaitCommand(time));
    }

    public Command setLEDs(int r, int g, int b) {
        return this.setLEDs(r, g, b, -1);
    }
}
