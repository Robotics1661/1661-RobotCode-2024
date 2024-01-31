// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.GamePieceMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */ //22 5/8 in
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.574675; // DONE Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */ //22 7/8 in
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.581025; // DONE Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 31; // DONE Set Pigeon ID ~~(NO pigeon, use navx)~~ we are using pigeon now

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; // DONE Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 17; // DONE Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23; // DONE Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(117.94921875000001); // DONE Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 19; // DONE Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // DONE Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24; // DONE Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(116.3671875); // DONE Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; // DONE Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // DONE Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22; // DONE Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(27.7734375); // DONE Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; // DONE Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12; // DONE Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21; // DONE Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(154.599609375); // DONE Measure and set back right steer offset

    public static final int PCM = 30; //pneumatics control module
    public static final int ELBOW_MOTOR = 40;
    public static final int SHOULDER_MOTOR = 41;
    public static final double ELBOW_MAX_SPEED = -0.35; //percent (flipped)
    public static final double SHOULDER_MAX_SPEED = -0.35; //percent (flipped)

    public static final int SOLENOID_6_INCH_FORWARD = 2;
    public static final int SOLENOID_6_INCH_REVERSE = 3;
    public static final int SOLENOID_7_INCH_FORWARD = 6;
    public static final int SOLENOID_7_INCH_REVERSE = 7;

    public static final int CONE_BUTTON = 5;
    public static final int CUBE_BUTTON = 6;
    public static final int RELEASE_BUTTON = 2;

    public static final AutonomousMode AUTONOMOUS_MODE = AutonomousMode.FULL_20_POINT;
    public static final GamePieceMode GAME_PIECE_MODE = GamePieceMode.CUBE; // which game piece Full20Point auto should use
}
