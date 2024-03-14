package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RunnableMotorSafety;
import frc.robot.constants.Constants.CandleConstants;
import frc.robot.constants.Intake.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;

public class IntakeSubsystem extends SubsystemBase {
  /*-------------------------------- Private Instance Variables --------------------------------*/
  // Subsystem State
  private IntakeState curState = IntakeState.OFF;
  private IntakeIO hardwImpl;
  private RunnableMotorSafety systemMotorSafety = new RunnableMotorSafety(
      () -> this.setIntakeState(IntakeState.OFF),
      "Intake");
  private LEDSubsystem leds;

  private Debouncer ledDebouncer = new Debouncer(0.25);
  private Command ledCommand;

  public IntakeSubsystem(IntakeIO hardwImpl, LEDSubsystem leds) {
    this.hardwImpl = hardwImpl;
    this.leds = leds;
    this.ledCommand = this.leds.animate(new SingleFadeAnimation(0x33, 0x66, 0xff, 0, 0.5, CandleConstants.kLEDCount));
    this.initializeShuffleboardWidgets();
  }

  public static enum IntakeState {
    INTAKE(IntakeConstants.kIntakeInSpeed, IntakeConstants.kIndexInSpeed), // Intake full in
    INTAKEANDHOLD(IntakeConstants.kIntakeOutSpeed, 0f), // Intake out index stop
    FEED(0f, IntakeConstants.kIndexInSpeed), // Intake off index full in
    OUTAKEANDHOLD(IntakeConstants.kIntakeOutSpeed, 0f), // Intake full out
    EJECT(IntakeConstants.kIntakeOutSpeed, IntakeConstants.kIndexOutSpeed), // Intake full out
    OFF(0f, 0f); // Intake full of

    public final double intakeSpeed, indexSpeed;

    private IntakeState(double intakespeed, double indexspeed) {
      this.intakeSpeed = intakespeed;
      this.indexSpeed = indexspeed;
    }
  }

  IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  @Override
  public void periodic() {
    Logger.processInputs("Intake", inputs);
    this.hardwImpl.updateInputs(inputs);
    Logger.recordOutput("Intake/curState", this.curState.name());
  
    if (this.ledDebouncer.calculate(this.getIndexSensor())) {
      this.leds.setDefaultCommand(this.ledCommand);
    } 
    else {
      this.leds.removeDefaultCommand();
      this.ledCommand.cancel();
    }
  }

  public void setIntakeState(IntakeState state) {
    this.systemMotorSafety.feed();
    this.hardwImpl.setIntakeSpeed(state.intakeSpeed);
    this.hardwImpl.setIndexSpeed(state.indexSpeed);
    if(state != curState) {
        curState = state;
    }
  }

  public Command setStateCommand(IntakeState state) {
    return new FunctionalCommand(
        () -> {},
        () -> this.setIntakeState(state),
        (interrupted) -> this.setIntakeState(IntakeState.OFF),
        () -> false,
        this);
  }

  public boolean getIndexSensor() {
    return this.hardwImpl.getIndexSensor();
  }

  public void feed() {
    this.systemMotorSafety.feed();
  }

  // Show subsystem status on the dashboard
  private void initializeShuffleboardWidgets() {
    // var layout = Shuffleboard.getTab("Robot").getLayout("Shooter",
    // BuiltInLayouts.kList);
    // layout.addString("State", curState::name);
    // layout.addBoolean("Feed Sensor", this::getIndexSensor);

    Shuffleboard.getTab("Robot").addString("Intake State", curState::name);
    Shuffleboard.getTab("Robot").addBoolean("Feeder Sensor", this::getIndexSensor);
  }
}
