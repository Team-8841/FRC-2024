package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.IntakeSubsytem.IntakeState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(OIConstants.gamepadPort);
    private final Joystick copilot = new Joystick(OIConstants.copilotPort);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton shootBtn = new JoystickButton(driver, OIConstants.buttonRB);
    private final JoystickButton visionAssistBtn = new JoystickButton(driver, OIConstants.buttonLB);
    private final JoystickButton zeroGyroBtn = new JoystickButton(driver, OIConstants.buttonSel);
    private final double hoodTrigger = driver.getRawAxis(3);

    /* Copilot Buttons */
    private final JoystickButton compressorOverrideBtn = new JoystickButton(copilot, OIConstants.button1);
    private final JoystickButton hoodDeployBtn = new JoystickButton(copilot, OIConstants.button2);
    private final JoystickButton hoodOverrideBtn = new JoystickButton(copilot, OIConstants.button3);
    private final JoystickButton intakeInBtn = new JoystickButton(copilot, OIConstants.button4);
    private final JoystickButton intakeOutBtn = new JoystickButton(copilot, OIConstants.button5);
    private final JoystickButton shooterAngleBtn = new JoystickButton(copilot, OIConstants.button6);
    private final JoystickButton climberBreaksBtn = new JoystickButton(copilot, OIConstants.button7);
    private final double climberMovement = copilot.getRawAxis(OIConstants.analog1);

    /* Subsystems */
    private final static Swerve s_Swerve = new Swerve();
    private final IntakeSubsytem s_Intake = new IntakeSubsytem();
    //private final LEDSubsystem s_LED = new LEDSubsystem();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final VisionSubsystem s_Vision = new VisionSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();


    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);


    

    /* Auto Builder stuff */

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* Pathplanner Named Commands */
        NamedCommands.registerCommand("IntakeIn", new IntakeCommand(true, false, s_Intake.getIndexSensor(), s_Intake));
        NamedCommands.registerCommand("IntakeOff", new IntakeCommand(false, false, s_Intake.getIndexSensor(), s_Intake));



        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );

        s_Intake.setDefaultCommand(new RunCommand(() -> {
            if(intakeInBtn.getAsBoolean() && !intakeOutBtn.getAsBoolean()) {
                if(s_Intake.getIndexSensor()) {
                    s_Intake.setState(IntakeState.OFF);
                } else {
                    s_Intake.setState(IntakeState.INTAKE);
                }
            } else if(intakeInBtn.getAsBoolean() && intakeOutBtn.getAsBoolean()) {
                s_Intake.setState(IntakeState.OUTAKE);
            } else if (!intakeInBtn.getAsBoolean() && intakeOutBtn.getAsBoolean()) {
                s_Intake.setState(IntakeState.OUTAKE);
            }else {
                s_Intake.setState(IntakeState.OFF);
            }
        }, s_Intake));

        s_Shooter.setDefaultCommand(new RunCommand(() -> {

            s_Shooter.setShooterSpeed(getRequestedShooterSpeed());

            if(hoodDeployBtn.getAsBoolean()) {
                DriverStation.reportWarning("Hood should be working.", false);
                s_Shooter.setEEAngle(ShooterConstants.kEEDeployed);
            } else {
                s_Shooter.setEEAngle(ShooterConstants.kEEHome);
            }

            
        }, s_Shooter));

        s_Elevator.setDefaultCommand(new ElevatorCommand(climberMovement, s_Elevator));


        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyroBtn.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        visionAssistBtn.whileTrue(
                new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -s_Vision.getTravelSpeed(), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );

        shooterAngleBtn.onTrue(new InstantCommand(() -> s_Shooter.setShooterAngle(true)))
                       .onFalse(new InstantCommand(() -> s_Shooter.setShooterAngle(false)));
        

        climberBreaksBtn.onTrue(new InstantCommand(() -> s_Elevator.setBreaks(true)))
                        .onFalse(new InstantCommand(() -> s_Elevator.setBreaks(false)));

        shootBtn.whileTrue(new RunCommand(() -> {
            if(s_Shooter.isShooterAtSpeed() && s_Shooter.getShooterSetpoint() > 0) {
                s_Intake.setState(IntakeState.FEED);
            }
        }));

        compressorOverrideBtn.onTrue(new InstantCommand(() -> compressor.disable()))
                             .onFalse(new InstantCommand(() -> compressor.enableDigital()));


    }

    public double getRequestedShooterSpeed(){
        double shooterSpeed = copilot.getRawAxis(3);
        if(shooterSpeed == 1){
            return 0;
        } else if (shooterSpeed < 0.6 && shooterSpeed > 0.53) {
            return ShooterConstants.kAmpShotSpeed;
        } else if(shooterSpeed < 0.049 && shooterSpeed > 0.050) {
            return ShooterConstants.kSubShotSpeed;
        } else if (shooterSpeed < -0.43 && shooterSpeed > -0.46){
            return ShooterConstants.kFarrShotSpeed1;
        }else{
            return 0;
        }
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Blue_2Note_Auto");
    }
}
