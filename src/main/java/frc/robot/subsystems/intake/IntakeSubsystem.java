package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    OFF(0f, 0f);                                      // Intake full off

    public final double intakeSpeed, indexSpeed;

    private IntakeState(double intakespeed, double indexspeed) {
      this.intakeSpeed = intakespeed;
      this.indexSpeed = indexspeed;
    }
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  /*-------------------------------- Custom Public Functions --------------------------------*/

  // Set intake state
  public void setIntakeState(IntakeState state) {
    curState = state;
    this.setIntakeSpeed(state.intakeSpeed);
    this.setIndexSpeed(state.indexSpeed);
  }

  // Set intake motor speed
  public void setIntakeSpeed(double speed) {
    this.hardwImpl.setIntakeSpeed(speed);
  }

  // set index motor speed
  public void setIndexSpeed(double speed) {
    this.hardwImpl.setIndexSpeed(speed);
  }

  // Get the index sensor
  public boolean getIndexSensor() {
    return this.hardwImpl.getIndexSensor();
  }

  /*-------------------------------- Custom Private Functions --------------------------------*/

  // Show subsystem status on the dashboard
  private void initializeShuffleboardWidgets() {
    var tab = Shuffleboard.getTab("Robot").getLayout("Intake");
    tab.addString("State", curState::name);
    tab.addBoolean("Feed Sensor", this::getIndexSensor);
  }
}
