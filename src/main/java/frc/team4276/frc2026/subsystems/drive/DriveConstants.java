package frc.team4276.frc2026.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2026.Ports;

public class DriveConstants {
    public static final int odometryFrequency = 100;
    public static final double trackWidth = Units.inchesToMeters(19.5);
    public static final double wheelBase = Units.inchesToMeters(27.5);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

    public static final double maxVelocityMPS = 5.632;
    public static final double maxAngularVelocity = 13.154;

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d[] zeroRotations = {
            Rotation2d.fromDegrees(98), // FL
            Rotation2d.fromDegrees(171), // FR
            Rotation2d.fromDegrees(2.5), // BL
            Rotation2d.fromDegrees(149) // BR
    };

    public static final Rotation2d[] zeroHelperRotations = {
            Rotation2d.kCCW_90deg, // FL
            Rotation2d.kZero, // FR
            Rotation2d.k180deg, // BL
            Rotation2d.kCW_90deg // BR
    };

    // Device CAN IDs
    public static final int[][] canIds = {
            { Ports.FRONT_LEFT_DRIVE, Ports.FRONT_LEFT_TURN },
            { Ports.FRONT_RIGHT_DRIVE, Ports.FRONT_RIGHT_TURN },
            { Ports.BACK_LEFT_DRIVE, Ports.BACK_LEFT_TURN },
            { Ports.BACK_RIGHT_DRIVE, Ports.BACK_RIGHT_TURN } };

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 60;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.47);
    public static final double drivingMotorPinionTeeth = 14.0;
    public static final double driveMotorReduction = (45.0 * 22.0) / (drivingMotorPinionTeeth * 15.0);

    public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);
    public static final double maxSteerVelocity = driveGearbox.freeSpeedRadPerSec / driveMotorReduction;

    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction;

    

    // Drive PID configuration
    public static final double driveKp = 0.006;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.03051;
    public static final double driveKv = 0.0901;
    public static final double driveKa = 0.004;
    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 1.0;
    public static final double turnKd = 0.0;

    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
}
