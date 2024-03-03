package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CandleConstants;

public class LEDSubsystem extends SubsystemBase {
  /*-------------------------------- Private Instance Variables --------------------------------*/
  private CANdle m_candle = new CANdle(CandleConstants.kCandleID, "rio");

  private static int m_LEDCount = CandleConstants.kLEDCount;

  private Animation m_toAnimate = null;
  private AnimationTypes m_currentAnimation;

  

  /*-------------------------------- Public Instance Variables --------------------------------*/

  public enum AnimationTypes {
      ColorFlow,
      Fire,
      Larson,
      Rainbow,
      RgbFade,
      SingleFade,
      Strobe,
      StrobeRed,
      StrobeGreen,
      Twinkle,
      TwinkleOff,
      SetAll
  }

  public LEDSubsystem() {
    configBrightness(CandleConstants.kMaxBrightness);
    configLedType(LEDStripType.RGB);
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    updateStatus();

  }

/*-------------------------------- Custom Public Functions --------------------------------*/
  public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        if(m_currentAnimation != toChange){// Prevent spam changing the led animation
            m_currentAnimation = toChange;
            
            switch(toChange)
            {
                case ColorFlow:
                    m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, m_LEDCount, Direction.Forward);
                    break;
                case Fire:
                    m_toAnimate = new FireAnimation(0.5, 0.7, m_LEDCount, 0.7, 0.5);
                    break;
                case Larson:
                    m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, m_LEDCount, BounceMode.Front, 3);
                    break;
                case Rainbow:
                    m_toAnimate = new RainbowAnimation(1, 0.1, m_LEDCount);
                    break;
                case RgbFade:
                    m_toAnimate = new RgbFadeAnimation(0.7, 0.4, m_LEDCount);
                    break;
                case SingleFade:
                    m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, m_LEDCount);
                    break;
                case Strobe:
                    m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, m_LEDCount);
                    break;
                case StrobeRed:
                    m_toAnimate = new StrobeAnimation(255, 0, 0, 0, 98.0 / 256.0, m_LEDCount);
                    break;
                case StrobeGreen:
                    m_toAnimate = new StrobeAnimation(0, 255, 0, 0, 98.0 / 256.0, m_LEDCount);
                    break;
                case Twinkle:
                    m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, m_LEDCount, TwinklePercent.Percent6);
                    break;
                case TwinkleOff:
                    m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, m_LEDCount, TwinkleOffPercent.Percent100);
                    break;
                case SetAll:
                    m_toAnimate = null;
                    break;
            }
            System.out.println("Changed to " + m_currentAnimation.toString());
        }
    }

  /*-------------------------------- Custom Private Functions --------------------------------*/

    private void updateStatus() {
        SmartDashboard.putNumber("[CANdle]: VBus", getVbat());
        SmartDashboard.putNumber("[CANdle]: 5V Rail", get5V());
        SmartDashboard.putNumber("[CANdle]: Current", getCurrent());
        SmartDashboard.putNumber("[CANdle]: Temp", getTemperature());
    }

}

