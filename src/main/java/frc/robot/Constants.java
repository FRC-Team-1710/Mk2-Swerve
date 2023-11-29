// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;

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



public class Constants {

    public static class Vision {
        public static final String kCameraName = "christiansLeftEye";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = errorWrapper();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
    
    private static AprilTagFieldLayout errorWrapper() {
        try {
            AprilTagFieldLayout attemptedKTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            return attemptedKTagLayout;
        } catch (Exception e) {
            return null;
        }
    }

    // DRIVE CONSTANTS
    public static final double DRIVETRAIN_MAX_SPEED = 0.75;

    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2; // CAN
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; // Analog
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 3; // CAN

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 8; // CAN
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 3; // Analog
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 9; // CAN

    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 4; // CAN
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 1; // Analog
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 5; // CAN

    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 6; // CAN
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 2; // Analog
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 7; // CAN


}
