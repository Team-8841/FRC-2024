package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*-------------------------------- Private Instance Variables --------------------------------*/

    private ShooterIO hwImpl;

    private ShooterState curState;

    private ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    /*-------------------------------- Public Instance Variables --------------------------------*/

    public static enum ShooterState {
        /*
         * ENUM(shooter Setpoint, End effector setpoint, End effector roller speed
         * ,Shooter Aimed Up)
         */
        OFF(ShooterConstants.shooterOffSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed, true),
        SUBSUBSHOT(ShooterConstants.subSubShotSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed, true),
        SUBSHOT(ShooterConstants.subShotSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed, true),
        FARSHOT(ShooterConstants.farShotSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed,
                false),
        FARFARSHOT(ShooterConstants.farFarShotSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed,
                false),
        AMP(ShooterConstants.ampShotSpeed, ShooterConstants.endEffectorDeployed, ShooterConstants.rollerOutSpeed, true),
        FIELDTOSS(ShooterConstants.fieldTossSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed,
                false);

        private final double m_shooterSP, m_endEffectorRollerSpeed;
        private final Rotation2d m_endEffectorSP;

        private ShooterState(double shooterSP, Rotation2d endEffectorSP, double eeRollerSpeed, boolean shooterAimedUP) {
            this.m_shooterSP = shooterSP;
            this.m_endEffectorSP = endEffectorSP;
            this.m_endEffectorRollerSpeed = eeRollerSpeed;
        }

    }

    public ShooterSubsystem(ShooterIO hwImpl) {
        this.hwImpl = hwImpl;
        this.initializeShuffleboardWidgets();
    }
    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
        this.hwImpl.updateInputs(this.inputs);
        Logger.processInputs("/Shooter", this.inputs);
    }

    /*-------------------------------- Custom Public Functions --------------------------------*/

    private void setShooterState(ShooterState state) {
        this.setShooterSetPoint(state.m_shooterSP);
        this.setEndEffector(state.m_endEffectorSP);
        this.setRollerSpeed(state.m_endEffectorRollerSpeed);
        curState = state;
    }

    /* ========= Shooter Functions ========= */
    public Command setStateCommand(ShooterState state) {
        return new FunctionalCommand(
            () -> this.setShooterState(state),
            () -> {},
            (interrupted) -> this.setShooterState(ShooterState.OFF),
            () -> false,
            this
        );
    }

    private void setShooterSetPoint(double targetRPS) {
        this.hwImpl.setShooter(targetRPS);
    }

    // Get the target setpoint from the current state and check if we are within a
    // tolerance of that value
    public boolean shooterAtTarget() {
        return this.hwImpl.getShooterRPS() > curState.m_shooterSP - ShooterConstants.shooter_allowedError;
    }

    /* ========= End Effector Functions ========= */
    private void setEndEffector(Rotation2d targetAngle) {
        this.hwImpl.setEndEffector(targetAngle);
    }

    // Get the target setpoint from the current state and check if we are within a
    // tolerance of that value
    public boolean endEffectorAtTarget() {
        double angle = this.hwImpl.getEndEffector().getDegrees();
        double spAngle = curState.m_endEffectorSP.getDegrees();
        double allowedError = ShooterConstants.endEffector_allowedError.getDegrees();
        return Math.abs(spAngle - angle) < allowedError;
    }

    private void setRollerSpeed(double dcycle) {
        this.hwImpl.setRollerSpeed(dcycle);
    }

    /*-------------------------------- Custom Private Functions --------------------------------*/
    private void initializeShuffleboardWidgets() {
        var layout = Shuffleboard.getTab("Robot").getLayout("Shooter", BuiltInLayouts.kList);
        layout.addDouble("Velocity", this.hwImpl::getShooterRPS);
        layout.addBoolean("At Speed", this::shooterAtTarget);
        layout.addDouble("Angle", () -> this.hwImpl.getEndEffector().getDegrees());
        layout.addBoolean("At Target", this::endEffectorAtTarget);
    }

}
