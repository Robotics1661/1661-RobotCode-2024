// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double SWERVE_MAX_SPEED = 6; // 6 meters per second desired top speed
    public static double SWERVE_MAX_ANGULAR_RATE = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    //public static final int PCM = 30; //pneumatics control module

    public static final int INTAKE_ID = 36;

    public static final int FOUR_BAR_RIGHT_ID = 41;
    public static final int FOUR_BAR_LEFT_ID = 40;

    public static final int FOUR_BAR_ENCODER_RIGHT_ID = 25;
    public static final int FOUR_BAR_ENCODER_LEFT_ID = 26;
    
    public static final int SHOOTER_FRONT_ID = 33;
    public static final int SHOOTER_BACK_ID = 34;

    public static final int CLIMBER_ID = 35;

    public static final HolonomicPathFollowerConfig PATH_FOLLOW_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        4.5, // Max module speed, in m/s
        0.558614, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
    );

    public static final AutonomousMode AUTONOMOUS_MODE = AutonomousMode.SPEAKER_SHOT_AND_INTAKE_MIDDLE_AND_SHOOT_FAR;//.SPEAKER_SHOT_AND_INTAKE_MIDDLE_AND_RACE_FORWARD;

    public static final boolean ENABLE_LL_VISION_ESTIMATE = true;
    public static final boolean FOUR_BAR_POSITION_HOLD_ASSIST = true;

    // WARN: make sure to set to false before any matches
    public static final boolean DISABLE_SWERVE = false;
    public static final boolean DISABLE_CLIMBER = true;
    public static final boolean DISABLE_AUTO_PUSH_FORWARD = false;
}
