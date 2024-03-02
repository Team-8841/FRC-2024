package frc.robot.constants;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.constants.swerve.SwerveConstants;

public final class PathingConstants {
    public static final PathPlannerPath basicPath = PathPlannerPath.fromPathFile("basicPath");

    public static final ReplanningConfig replanningConfig = new ReplanningConfig(false, false, 1, 1);
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        5, 
        SwerveConstants.compModulePositions[0].getNorm(), 
        PathingConstants.replanningConfig
    );

    public static final PIDConstants translationConstants = new PIDConstants(1), rotationConstants = new PIDConstants(1);

    public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(translationConstants, rotationConstants, SwerveConstants.maxSpeed, 0);
}
