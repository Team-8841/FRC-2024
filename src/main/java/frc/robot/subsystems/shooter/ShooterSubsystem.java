package frc.robot.subsystems.shooter;

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
    private RunnableMotorSafety systemMotorSafety = new RunnableMotorSafety(
        () -> this.setShooterState(ShooterState.OFF), 
        "Shooter"
    );
    private boolean lastLimitSwitch;

    /*-------------------------------- Public Instance Variables --------------------------------*/

    public static enum ShooterState {
        /*
         * ENUM(shooter Setpoint, End effector setpoint, End effector roller speed
         * ,Shooter Aimed Up)
         */
        OFF(0, ShooterConstants.endEffectorHome, 0, true),
        SUBSHOT(ShooterConstants.subShotSpeed, ShooterConstants.endEffectorHome, 0, true),
        FARSHOT(ShooterConstants.farShotSpeed1, ShooterConstants.endEffectorHome, 0,
                false),
        AMP(ShooterConstants.ampShotSpeed, ShooterConstants.endEffectorDeployed, ShooterConstants.rollerOutSpeed, true),
        FIELDTOSS(ShooterConstants.farShotSpeed2, ShooterConstants.endEffectorHome, 0,
                false),

        COOLAMP(0, Rotation2d.fromDegrees(180), 0, false),
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
        boolean limitSwitch = this.hwImpl.getLimitSwitch();
        if (!this.lastLimitSwitch && limitSwitch) {
            System.out.println("End Effector Limit switch engaged");
        }
        else if (this.lastLimitSwitch && !limitSwitch) {
            System.out.println("End Effector Limit switch disengaged");
        }
        this.lastLimitSwitch = limitSwitch;

        if (limitSwitch) {
            this.hwImpl.endEffectorLimit();
        }

        this.hwImpl.updateInputs(this.inputs);
        Logger.processInputs("Shooter", this.inputs);

        this.latestFilteredRPS = this.rpsFilter.calculate(this.hwImpl.getShooterRPS());
        Logger.recordOutput("Shooter/filteredRPS", -this.latestFilteredRPS);

        Logger.recordOutput("Shooter/curState", this.curState.name());
    }

    public void setShooterState(ShooterState state) {
        this.systemMotorSafety.feed();
        this.hwImpl.setShooter(state.m_shooterDcycle);
        this.hwImpl.setEndEffector(state.m_endEffectorSP);
        this.hwImpl.setRollerSpeed(state.m_endEffectorRollerDcycle);
        curState = state;
    }

    public Command setStateCommand(ShooterState state) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                // System.out.println(state.name());
                this.setShooterState(state);
            },
            (interrupted) -> this.setShooterState(ShooterState.OFF),
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
        this.systemMotorSafety.feed();
    }

    private void initializeShuffleboardWidgets() {
        //var layout = Shuffleboard.getTab("Robot").getLayout("Shooter", BuiltInLayouts.kList);
        //layout.addDouble("Velocity", this.hwImpl::getShooterRPS);
        //layout.addBoolean("At Speed", this::shooterAtTarget);
        //layout.addDouble("Angle", () -> this.hwImpl.getEndEffector().getDegrees());
        //layout.addBoolean("At Target", this::endEffectorAtTarget);
    }

}
