// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Locale;

import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

    //public static final int PCM = 30; //pneumatics control module

    public static final AutonomousMode AUTONOMOUS_MODE = AutonomousMode.NONE;

    public static enum SwerveModuleConfig {
        FRONT_LEFT(
            11,
            17,
            23,
            -Math.toRadians(117.94921875000001), // Steer offset
            0, 0
        ),
        FRONT_RIGHT(
            19,
            7,
            24,
            -Math.toRadians(116.3671875), // Steer offset
            2, 0
        ),
        BACK_LEFT(
            6,
            8,
            22,
            -Math.toRadians(27.7734375), // Steer offset
            4, 0
        ),
        BACK_RIGHT(
            5,
            12,
            21,
            -Math.toRadians(154.599609375), // Steer offset
            6, 0
        )
        ;
        public final int driveMotor;
        public final int steerMotor;
        public final int steerEncoder;
        public final double steerOffset;
        private final int shuffleboardColumn;
        private final int shuffleboardRow;
        private boolean moduleInitialized = false;

        private SwerveModuleConfig(int driveMotor, int steerMotor, int steerEncoder, double steerOffset,
                int shuffleboardColumn, int shuffleboardRow) {
            this.driveMotor = driveMotor;
            this.steerMotor = steerMotor;
            this.steerEncoder = steerEncoder;
            this.steerOffset = steerOffset;
            this.shuffleboardColumn = shuffleboardColumn;
            this.shuffleboardRow = shuffleboardRow;
        }

        private String humanName() {
            String[] pieces = this.name().split("_");
            StringBuilder out = new StringBuilder();
            boolean first = true;
            for (String piece : pieces) {
                if (first) {
                    first = false;
                } else {
                    out.append(' ');
                }
                out.append(piece.substring(0, 1).toUpperCase(Locale.ROOT));
                out.append(piece.substring(1).toLowerCase(Locale.ROOT));
            }
            return out.toString();
        }

        public SwerveModule create() {
            if (moduleInitialized) {
                System.out.println("WARNING: Swerve module {} initalized twice!".formatted(this));
            }
            moduleInitialized = true;

            ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

            return new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                // ID of the drive motor
                .withDriveMotor(MotorType.FALCON, driveMotor)
                // ID of the steer motor
                .withSteerMotor(MotorType.FALCON, steerMotor)
                // ID of the steer encoder
                .withSteerEncoderPort(steerEncoder)
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                .withSteerOffset(steerOffset)
                // Shuffleboard state
                .withLayout(tab
                        .getLayout(this.humanName() + " Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(shuffleboardColumn, shuffleboardRow)
                ).build();
        }
    }
}
