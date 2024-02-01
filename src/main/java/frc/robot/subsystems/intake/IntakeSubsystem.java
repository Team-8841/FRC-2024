package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Intake.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /*-------------------------------- Private Instance Variables --------------------------------*/
  // Subsystem State
  private IntakeState curState;
  private IntakeIO hardwImpl;

  public IntakeSubsystem(IntakeIO hardwImpl) {
    this.hardwImpl = hardwImpl;
    this.initializeShuffleboardWidgets();
  

  }

  /*-------------------------------- Public Instance Variables --------------------------------*/

  public static enum IntakeState {
    INTAKE(IntakeConstants.kIntakeInSpeed, IntakeConstants.kIndexInSpeed),   // Intake full in
    INTAKEANDHOLD(IntakeConstants.kIntakeOutSpeed, 0f),           // Intake out index stop
    FEED(0f, IntakeConstants.kIndexInSpeed),                     // Intake off index full in 
    OUTAKE(IntakeConstants.kIntakeOutSpeed, IntakeConstants.kIndexOutSpeed), // Intake full out
    OFF(0f, 0f);                                      // Intake full of

    public final double intakeSpeed, indexSpeed;

    private IntakeState(double intakespeed, double indexspeed) {
      this.intakeSpeed = intakespeed;
      this.indexSpeed = indexspeed;
    }
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  @Override
  public void periodic() {
    Logger.processInputs("/Intake", inputs);
    this.hardwImpl.updateInputs(inputs);
  }

  /*-------------------------------- Custom Public Functions --------------------------------*/

  public void setIntakeState(IntakeState state) {
    curState = state;
    this.setIntakeSpeed(state.intakeSpeed);
    this.setIndexSpeed(state.indexSpeed);
  }

  public void setIntakeSpeed(double speed) {
    this.hardwImpl.setIntakeSpeed(speed);
  }

  public void setIndexSpeed(double speed) {
    this.hardwImpl.setIndexSpeed(speed);
  }

  public boolean getIntakeSensor() {
    return this.hardwImpl.getIntakeSensor();
  }

  public boolean getIndexSensor() {
    return this.hardwImpl.getIndexSensor();
  }

  public Command setStateCommand(IntakeState state) {
    return new FunctionalCommand(
      () -> this.setIntakeState(state),
      () -> {},
      (interrupted) -> this.setIntakeState(IntakeState.OFF),
      () -> false,
      this
    );
  }

  public Command sensorFeedCommand() {
    return new RunCommand(() -> {
        if (this.getIndexSensor()) {
          this.setIntakeState(IntakeState.INTAKEANDHOLD);
        }
        else if (this.getIntakeSensor()) {
          this.setIntakeState(IntakeState.INTAKE);
        }
        else {
          this.setIntakeState(IntakeState.OFF);
        }
      }, this)
      .finallyDo(() -> this.setIntakeState(IntakeState.OFF));
  }

  /*-------------------------------- Custom Private Functions --------------------------------*/

  // Show subsystem status on the dashboard
  private void initializeShuffleboardWidgets() {
    var layout = Shuffleboard.getTab("Robot").getLayout("Shooter", BuiltInLayouts.kList);
    layout.addString("State", curState::name);
    layout.addBoolean("Feed Sensor", this::getIndexSensor);
  }
}
