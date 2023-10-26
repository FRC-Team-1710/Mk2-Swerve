// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utilities.drivers.Gyroscope;
import frc.utilities.drivers.SwerveModule;
import frc.utilities.math.Vector2;
import frc.utilities.drivers.Mk2SwerveModuleBuilder;
import frc.utilities.drivers.NavX;

public class DriveSubsystem extends SubsystemBase {

    private static final double TRACKWIDTH = Units.inchesToMeters(23);
    private static final double WHEELBASE = Units.inchesToMeters(23);

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(216 - 90);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(337 - 90);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(330 - 90);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(114 - 90);

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
                    .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
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

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);
    public ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    /*private NetworkTableEntry backLeft;
    private NetworkTableEntry backRight;
    private NetworkTableEntry frontLeft;
    private NetworkTableEntry frontRight;
    private NetworkTableEntry gyroRot;*/

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

       /*frontRight = tab.add("Front Right Angle", 0).getEntry();
        frontLeft = tab.add("Front Left Angle", 0).getEntry();
        backRight = tab.add("Back Right Angle", 0).getEntry();
        backLeft = tab.add("Back Left Angle", 0).getEntry();
        gyroRot = tab.add("Gyro Angle", 0).getEntry(); */
    }

    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();
        // Outputs encoder values to Shuffleboard
        /*frontRight.setDouble(Math.toDegrees(frontRightModule.getCurrentAngle()));
        frontLeft.setDouble(Math.toDegrees(frontLeftModule.getCurrentAngle()));
        backRight.setDouble(Math.toDegrees(backRightModule.getCurrentAngle()));
        backLeft.setDouble(Math.toDegrees(backLeftModule.getCurrentAngle()));

        gyroRot.setDouble(gyroscope.getAngle().toDegrees());*/

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);

        SmartDashboard.putNumber("Front Left", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right", Math.toDegrees(backRightModule.getCurrentAngle()));
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
