package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Intake.IntakeConstants;

public class IntakeSubsytem extends SubsystemBase {



  /*-------------------------------- Private Instance Variables --------------------------------*/

  // Motors
  private final CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.intakeMotor, MotorType.kBrushed);
  private final CANSparkMax indexMotor =
      new CANSparkMax(IntakeConstants.indexMotor, MotorType.kBrushed);

  // Sensors
  private DigitalInput indexSenor = new DigitalInput(IntakeConstants.indexSensor);

  // Subsystem State
  private IntakeState curState;

  public IntakeSubsytem() {}

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

  @Override
  public void periodic() {
    updateStatus();
  }

  /*-------------------------------- Custom Public Functions --------------------------------*/

  // Set intake state
  public void intake(IntakeState state) {
    curState = state;
    setIntakeSpeed(state.intakeSpeed);
    setIndexSpeed(state.indexSpeed);
  }

  // Set intake motor speed
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  // set index motor speed
  public void setIndexSpeed(double speed){
    indexMotor.set(speed);
  }

  // Get the index sensor
  public boolean getIndexSensor() {
    return indexSenor.get();
  }

  // Show subsystem status on the dashboard
  private void updateStatus() {
    SmartDashboard.putString("[Intake]: Current State ", curState.name());
    SmartDashboard.putBoolean("[Intake] Feed Sensor", getIndexSensor());
  }

  /*-------------------------------- Custom Private Functions --------------------------------*/
}
