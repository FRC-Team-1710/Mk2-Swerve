// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;

import java.util.Random;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utilities.drivers.Gyroscope;
import frc.utilities.drivers.SwerveModule;
import frc.utilities.math.Vector2;
import frc.utilities.drivers.Mk2SwerveModuleBuilder;
import frc.utilities.drivers.NavX;

public class DriveSubsystem extends SubsystemBase {

        private final SwerveDrivePoseEstimator poseEstimator;
        private Vision vision;

        private static final double TRACKWIDTH = Units.inchesToMeters(23);
        private static final double WHEELBASE = Units.inchesToMeters(23);

        private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(220 - 90);
        private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(338 - 90);
        private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(329 - 90);
        private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(114 - 90);

        private Random rand = new Random(4512);

        private CANSparkMax backLeftAngle = new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backRightAngle = new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backLeftDrive = new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backRightDrive = new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontLeftAngle = new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontRightAngle = new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontLeftDrive = new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontRightDrive = new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);

        /** Front left swerve module object */
        private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
                        new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER),
                                        FRONT_LEFT_ANGLE_OFFSET)
                        .angleMotor(frontLeftAngle,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .driveMotor(frontLeftDrive,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .build();
        /** Front right swerve module object */
        private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
                        new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER),
                                        FRONT_RIGHT_ANGLE_OFFSET)
                        .angleMotor(frontRightAngle,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .driveMotor(frontRightDrive,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .build();
        /** Back left swerve module object */
        private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
                        new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER),
                                        BACK_LEFT_ANGLE_OFFSET)
                        .angleMotor(backLeftAngle,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .driveMotor(backLeftDrive,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .build();
        /** Back right swerve module object */
        private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
                        new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER),
                                        BACK_RIGHT_ANGLE_OFFSET)
                        .angleMotor(backRightAngle,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .driveMotor(backRightDrive,
                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                        .build();

        /** Ratios for swerve calculations */
        public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                        new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                        new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                        new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

        public Rotation2d getGyroYaw() {
                return new Rotation2d(gyroscope.getAngle().toRadians());
        }

        public Pose2d getPose() {
                return poseEstimator.getEstimatedPosition();
        }

        private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);
        public ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        /*
         * private NetworkTableEntry backLeft;
         * private NetworkTableEntry backRight;
         * private NetworkTableEntry frontLeft;
         * private NetworkTableEntry frontRight;
         * private NetworkTableEntry gyroRot;
         */

        /** Creates a new DriveSubsystem. */
        public DriveSubsystem() {
                gyroscope.calibrate();
                gyroscope.setInverted(true); // You might not need to invert the gyro

                backLeftAngle.setSmartCurrentLimit(20);
                backRightAngle.setSmartCurrentLimit(20);
                backLeftDrive.setSmartCurrentLimit(20);
                backRightDrive.setSmartCurrentLimit(20);
                frontLeftAngle.setSmartCurrentLimit(20);
                frontLeftDrive.setSmartCurrentLimit(20);
                frontRightAngle.setSmartCurrentLimit(20);
                frontRightDrive.setSmartCurrentLimit(20);

                // Use addRequirements() here to declare subsystem dependencies.
                frontLeftModule.setName("Front Left");
                frontRightModule.setName("Front Right");
                backLeftModule.setName("Back Left");
                backRightModule.setName("Back Right");

                // and how many or how frequently vision measurements are applied to the pose
                // estimator.
                Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // Encoder Odometry
                Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1); // Vision Odometry

                poseEstimator = new SwerveDrivePoseEstimator(
                                kinematics,
                                getGyroYaw(),
                                getModulePositions(),
                                new Pose2d(),
                                stateStdDevs,
                                visionStdDevs);

                vision = new Vision();

                /*
                 * frontRight = tab.add("Front Right Angle", 0).getEntry();
                 * frontLeft = tab.add("Front Left Angle", 0).getEntry();
                 * backRight = tab.add("Back Right Angle", 0).getEntry();
                 * backLeft = tab.add("Back Left Angle", 0).getEntry();
                 * gyroRot = tab.add("Gyro Angle", 0).getEntry();
                 */
        }

        public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
                poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
        }

        /**
         * See
         * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
         */
        public void addVisionMeasurement(
                        Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
                poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
        }

        public void applyRandomOffset(DriveSubsystem drivetrain) {
                Transform2d trf = new Transform2d(
                                new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
                                new Rotation2d(rand.nextDouble() * 2 * Math.PI));
                drivetrain.resetPose(drivetrain.getPose().plus(trf));
        }

        public void resetPose(Pose2d pose) {
                poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }

        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] {
                                frontLeftModule.getPosition(),
                                frontRightModule.getPosition(),
                                backLeftModule.getPosition(),
                                backRightModule.getPosition()
                };
        }

        @Override
        public void periodic() {
                frontLeftModule.updateSensors();
                frontRightModule.updateSensors();
                backLeftModule.updateSensors();
                backRightModule.updateSensors();
                // Outputs encoder values to Shuffleboard
                /*
                 * frontRight.setDouble(Math.toDegrees(frontRightModule.getCurrentAngle()));
                 * frontLeft.setDouble(Math.toDegrees(frontLeftModule.getCurrentAngle()));
                 * backRight.setDouble(Math.toDegrees(backRightModule.getCurrentAngle()));
                 * backLeft.setDouble(Math.toDegrees(backLeftModule.getCurrentAngle()));
                 * 
                 * gyroRot.setDouble(gyroscope.getAngle().toDegrees());
                 */

                frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
                frontRightModule.updateState(TimedRobot.kDefaultPeriod);
                backLeftModule.updateState(TimedRobot.kDefaultPeriod);
                backRightModule.updateState(TimedRobot.kDefaultPeriod);

                SmartDashboard.putNumber("Front Left", Math.toDegrees(frontLeftModule.getCurrentAngle()));
                SmartDashboard.putNumber("Front Right", Math.toDegrees(frontRightModule.getCurrentAngle()));
                SmartDashboard.putNumber("Back Left", Math.toDegrees(backLeftModule.getCurrentAngle()));
                SmartDashboard.putNumber("Back Right", Math.toDegrees(backRightModule.getCurrentAngle()));
                // Correct pose estimate with vision measurements
                var visionEst = vision.getEstimatedGlobalPose();
                visionEst.ifPresent(
                                est -> {
                                        var estPose = est.estimatedPose.toPose2d();
                                        // Change our trust in the measurement based on the tags we can see
                                        var estStdDevs = vision.getEstimationStdDevs(estPose);

                                        addVisionMeasurement(
                                                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                                });

                
                if (visionEst.isPresent()) {
                        SmartDashboard.putString("obodom", visionEst.get().estimatedPose.toString());
                }
        
        }

        /**
         * Method for controlling all modules
         * 
         * <p>
         * The rotation value is multiplied by 2 and then divided by the hypotenuse of
         * the WHEELBASE and TRACKWIDTH.
         * The values of the forward, strafe, and rotation are outputted to
         * Shuffleboard.
         * The speed is then calculated using the ChassisSpeeds class.
         * Finally, the speeds are put into an array and set using
         * {@link #setTargetVelocity(speed, angle)}.
         * <p>
         * Also, the gyroscope is be reset here when the correct button is pressed
         * 
         * @param translation   The forward and strafe values sent through the
         *                      Translation2d class
         * @param rotation      The rotation value.
         * @param fieldOriented Boolean value that determines whether field orientation
         *                      is used
         *
         */
        public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
                rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
                rotation *= .5;
                SmartDashboard.putNumber("Left Joystick x", translation.getX());
                SmartDashboard.putNumber("Left Joystick y", translation.getY());
                SmartDashboard.putNumber("Rotation", rotation);

                ChassisSpeeds speeds;
                if (fieldOriented) {
                        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                                        Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
                } else {
                        speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
                }

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

                frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
                frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
                backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
                backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
        }

        public void resetGyroscope() {
                gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
        }
}