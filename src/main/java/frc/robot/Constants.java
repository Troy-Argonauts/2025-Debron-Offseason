// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public interface Controllers {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
        public static final double DEADBAND = 0.08;
    }

    public interface Climber {

        int LEFT_MOTOR_ID = 10;
        int RIGHT_MOTOR_ID = 11;
        int TONGUE_MOTOR_ID = 12;

        public static final double MAX_ARM_POSITION = -29.16;
        public static final double MAX_Tongue_POSITION = 2.4;

    }
    
    // THIS NEEDS TO BE UPDATED FOR CTRE
    public interface Swerve {
        // PID Constants for drivetrain
        public static final double DRIVE_P = 0.32;
        public static final double DRIVE_I = 1.6;
        public static final double DRIVE_D = 0.000075;
        public static final double DRIVE_S = 0.0;
        public static final double DRIVE_V = 0;
        public static final double DRIVE_A = 0.0;

        public static final double DRIVE_MM_ACCELERATION = 0;
        public static final double DRIVE_MM_JERK = 0;      

        // public static final double DRIVE_P = 2;
        // public static final double DRIVE_I = 0;
        // public static final double DRIVE_D = 0;
        // public static final double DRIVE_S = 0.0;
        // public static final double DRIVE_V = 0.25;

        // PID Constants for Turn
        public static final double TURN_P = 56.5; // 21
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.0;
        public static final double TURN_S = 0.0; // 20
        public static final double TURN_V = 0.0;
        public static final double TURN_A = 0.0;

        public static final double TURN_MM_CRUISE_VELOCITY = 0;
        public static final double TURN_MM_ACCELERATION = 0;
        public static final double TURN_MM_JERK = 0;

        public static final int DRIVE_MOTOR_PINION_TEETH = 14;
        public static final double WHEEL_DIAMETER_METERS = 0.1016;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = 6.746;

        public static final double drivingEncoderPositionFactor = (WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION;

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.724;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

        public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double TRACK_WIDTH = Units.inchesToMeters(21);
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(21);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -0.0166015625; // -0.25341796875 + Math.PI/4;
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0.171875; // 0.051513671875;
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = 0.2138671875; // -0.056884765625;
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = 0.0927734375; // 0.095947265625;

        // SPARK MAX CAN IDs
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 6;
        public static final int BACK_LEFT_DRIVING_CAN_ID = 8;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 5;
        public static final int BACK_RIGHT_DRIVING_CAN_ID = 2;

        public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
        public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
        public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;
        public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;

        public static final int FRONT_LEFT_TURNING_CAN_ID = 3;
        public static final int BACK_LEFT_TURNING_CAN_ID = 4;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 1;
        public static final int BACK_RIGHT_TURNING_CAN_ID = 7;

        public static final int FRONT_LEFT_CAN_ENCODER = 10;
        public static final int FRONT_RIGHT_CAN_ENCODER = 11;
        public static final int BACK_LEFT_CAN_ENCODER = 12;
        public static final int BACK_RIGHT_CAN_ENCODER = 9;

        public static final int PIGEON_CAN_ID = 0;

        public static final boolean GYRO_REVERSED = false;

        public static final String CANBUS_NAME = "Swerve CAN Bus";
        Translation2d[] moduleOffsets = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),};  // robot configs 2d translations
    }

    public interface SwerveModule {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = false;

        public static final int FREE_SPEED_RPM = 6000; // Free Speed RPM of Kraken

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60.0;
        public static final double WHEEL_DIAMETER_METERS = 0.102;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        // public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (Swerve.DRIVE_MOTOR_PINION_TEETH * 15);
        public static final double DRIVING_MOTOR_REDUCTION = 6.75;
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVE_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION; // meters

        public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians

        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second
        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 60; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public interface Elevator {
        int LEFT_MOTOR_ID = 1;
        int RIGHT_MOTOR_ID = 2;
        int INNER_BOTTOM_LIMIT_SWITCH_SLOT = 1;
        int OUTER_BOTTOM_LIMIT_SWITCH_SLOT = 5;

        int MOTION_MAGIC_CRUISE_VELOCITY = 25;
        int MOTION_MAGIC_ACCEL = 50;

        double P = 5.5;
        double I = 0.3;
        double D = 0.0005;
        double V = 0.16;
        double G = 0.5;
    }

    public interface Manipulator {

        public static final int TOP_MOTOR_CAN_ID = 4;
        public static final int BOTTOM_MOTOR_CAN_ID = 5;

        public static final int FUNNEL_BEAM_BREAK = 2;
        public static final int MANIPULATOR_BEAM_BREAK = 3;
    }

    public interface PathPlanner {
        double MOMENT_OF_INERTIA = 65.536;
        double ROBOT_MASS = 45.035372;
        double WHEEL_COF = 1.1;
    }
} // Evan is not cool. Ved is cool.
// I agree
