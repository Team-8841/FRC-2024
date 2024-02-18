package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*-------------------------------- Private Instance Variables --------------------------------*/

    private ShooterIO hwImpl;
    private ShooterState curState;
    private ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private LinearFilter rpsFilter = LinearFilter.movingAverage(5);
    private double latestFilteredRPS;

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
                false);

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
    }

    @Override
    public void periodic() {
        this.hwImpl.updateInputs(this.inputs);
        Logger.processInputs("Shooter", this.inputs);

        this.latestFilteredRPS = this.rpsFilter.calculate(this.hwImpl.getShooterRPS());
        Logger.recordOutput("Shooter/filteredRPS", this.latestFilteredRPS);
    }

    public void setShooterState(ShooterState state) {
        this.hwImpl.feed();
        this.hwImpl.setShooter(state.m_shooterDcycle);
        this.hwImpl.setEndEffector(state.m_endEffectorSP);
        this.hwImpl.setRollerSpeed(state.m_endEffectorRollerDcycle);
        curState = state;
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
        this.hwImpl.feed();
    }

    private void initializeShuffleboardWidgets() {
        var layout = Shuffleboard.getTab("Robot").getLayout("Shooter", BuiltInLayouts.kList);
        layout.addDouble("Velocity", this::getShooterRPS);
        layout.addDouble("Angle", () -> this.hwImpl.getEndEffector().getDegrees());
        layout.addBoolean("At Target", this::endEffectorAtTarget);
    }

}
