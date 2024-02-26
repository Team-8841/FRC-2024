package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RunnableMotorSafety;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*-------------------------------- Private Instance Variables --------------------------------*/

    private ShooterIO hwImpl;
    private ShooterState curState = ShooterState.OFF;
    private ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private LinearFilter rpsFilter = LinearFilter.movingAverage(5);
    private double latestFilteredRPS;
    private boolean upLimitSwitchPrev, downLimitSwitchPrev;

    /*-------------------------------- Public Instance Variables --------------------------------*/

    public static enum ShooterState {
        /*
         * ENUM(shooter Setpoint, End effector setpoint, End effector roller speed
         * ,Shooter Aimed Up)
         */
        OFF(0, Rotation2d.fromDegrees(60), 0, true),
        AMPRAISE(0, Rotation2d.fromDegrees(0), 0, false),
        AMPSHOT(0.15, Rotation2d.fromDegrees(0), 0.7, true),
        COOLSHOT(0.6, Rotation2d.fromDegrees(0), 0, true);

        private final double m_shooterDcycle, m_endEffectorRollerDcycle;
        private final Rotation2d m_endEffectorSP;

        private ShooterState(double shooterSP, Rotation2d endEffectorSP, double eeRollerSpeed, boolean shooterAimedUP) {
            this.m_shooterDcycle = shooterSP;
            this.m_endEffectorSP = endEffectorSP;
            this.m_endEffectorRollerDcycle = eeRollerSpeed;
        }

    }

    public ShooterSubsystem(ShooterIO hwImpl) {
        this.hwImpl = hwImpl;
        this.initializeShuffleboardWidgets();

        this.setShooterState(ShooterState.OFF);
    }

    @Override
    public void periodic() {
        // End Effector

        {
            // Upper limit switch
            boolean upLimitSwitch = this.hwImpl.getUpLimitSwitch();
            if (!this.upLimitSwitchPrev && upLimitSwitch) {
                this.hwImpl.setEndEffector(0);
                System.out.println("End effector up limit switch engaged");
            }
            else if (this.upLimitSwitchPrev && !upLimitSwitch) {
                System.out.println("End effector up limit switch disengaged");
            }
            this.upLimitSwitchPrev = upLimitSwitch;

            // Down limit switch
            boolean downLimitSwitch = this.hwImpl.getDownLimitSwitch();
            if (!this.downLimitSwitchPrev && downLimitSwitch) {
                this.hwImpl.setEndEffector(0);
                System.out.println("End effector down limit switch engaged");
            }
            else if (this.downLimitSwitchPrev && !downLimitSwitch) {
                System.out.println("End effector down limit switch disengaged");
            }
            this.downLimitSwitchPrev = downLimitSwitch;

            // Move logic

            double eeSP = this.curState.m_endEffectorSP.getDegrees();

            if (this.hwImpl.getDownLimitSwitch() && eeSP >= 30) {
                this.hwImpl.setEndEffector(0.4);
            }

            else if (this.hwImpl.getUpLimitSwitch() && eeSP < 30) {
                this.hwImpl.setEndEffector(-0.4);
            }
            else if (!this.hwImpl.getDownLimitSwitch() && !this.hwImpl.getUpLimitSwitch()) {
                if (eeSP >= 30) {
                    this.hwImpl.setEndEffector(0.4);
                }
                else {
                    this.hwImpl.setEndEffector(-0.4);;
                }
            }
            else if (this.hwImpl.getDownLimitSwitch() || this.hwImpl.getUpLimitSwitch()) {
                this.hwImpl.setEndEffector(0);
            }
        }

        // Shooter

        {
            this.hwImpl.updateInputs(this.inputs);
            Logger.processInputs("Shooter", this.inputs);

            this.latestFilteredRPS = this.rpsFilter.calculate(this.hwImpl.getShooterRPS());
            Logger.recordOutput("Shooter/filteredRPS", -this.latestFilteredRPS);

            Logger.recordOutput("Shooter/curState", this.curState.name());
        }

        this.hwImpl.periodic();
    }

    public void setShooterState(ShooterState state) {
        Logger.recordOutput("Shooter/endEffectorSP", this.curState.m_endEffectorSP.getDegrees());

        this.hwImpl.setShooter(curState.m_shooterDcycle);
        this.hwImpl.setRollerSpeed(curState.m_endEffectorRollerDcycle);

        // end effector is handled in `periodic`

        this.curState = state;
    }

    public Command setStateCommand(ShooterState state) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                // System.out.println(state.name());
                this.setShooterState(state);
            },
            (interrupted) -> this.setShooterState(state),
            () -> false,
            this);
    }

    public double getShooterRPS() {
        return this.latestFilteredRPS;
    }

    // Get the target setpoint from the current state and check if we are within a
    // tolerance of that value
    public boolean endEffectorAtTarget() {
        double angle = this.hwImpl.getEndEffector().getDegrees();
        double spAngle = curState.m_endEffectorSP.getDegrees();
        double allowedError = ShooterConstants.endEffector_allowedError.getDegrees();
        return Math.abs(spAngle - angle) < allowedError;
    }

    public void feed() {
    }

    private void initializeShuffleboardWidgets() {
        //var layout = Shuffleboard.getTab("Robot").getLayout("Shooter", BuiltInLayouts.kList);
        //layout.addDouble("Velocity", this.hwImpl::getShooterRPS);
        //layout.addBoolean("At Speed", this::shooterAtTarget);
        //layout.addDouble("Angle", () -> this.hwImpl.getEndEffector().getDegrees());
        //layout.addBoolean("At Target", this::endEffectorAtTarget);
    }

}
