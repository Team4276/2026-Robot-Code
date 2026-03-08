// Copyright (c) 2025-2026 Littleton Robotics
package frc.team4276.frc2026.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4276.frc2026.FieldConstants;
import frc.team4276.frc2026.RobotState;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.lib.dashboard.LoggedTunableNumber;
import frc.team4276.lib.geometry.AllianceFlipUtil;
import frc.team4276.lib.geometry.GeomUtil;

import static frc.team4276.frc2026.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
    private static ShotCalculator instance;

    private final LinearFilter robotHeadingFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

    private Rotation2d lastRobotAngle;
    private Rotation2d desiredRobotAngle;
    private double robotOmega;

    public static ShotCalculator getInstance() {
        if (instance == null)
            instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            boolean isValid,
            Rotation2d robotHeading,
            double robotOmega,
            double flywheelSpeed) {
    }

    // Cache parameters
    private ShootingParameters latestHubParameters = null;
    private ShootingParameters latestFerryParameters = null;

    private static LoggedTunableNumber minDistance = new LoggedTunableNumber("ShotCalculator/MinDistance", 1.5);
    private static LoggedTunableNumber maxDistance = new LoggedTunableNumber("ShotCalculator/MaxDistance", 2.5);
    private static double phaseDelay;
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        // minDistance = 1.34;
        // maxDistance = 5.60;
        phaseDelay = 0.03;

        shotFlywheelSpeedMap.put(1.34, 3000.0);
        shotFlywheelSpeedMap.put(5.60, 3000.0);

        timeOfFlightMap.put(5.60, 0.001);
        timeOfFlightMap.put(1.34, 0.001);
    }

    boolean stooop = true;

    public ShootingParameters getHubParameters() {
        if (latestHubParameters != null) {
            return latestHubParameters;
        }

        if (stooop) {
            return getShrimpleParameters();
        } 

        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
        ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getFieldVelocity();
        estimatedPose = estimatedPose.exp(
                new Twist2d(
                        robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate target
        Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Pose2d shooterPosition = estimatedPose.transformBy(GeomUtil.toTransform2d(robotToShooter));
        double shooterToTargetDistance = target.getDistance(shooterPosition.getTranslation());

        // Calculate field relative shooter velocity
        ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double robotVelocityX = robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToShooter.getY() * Math.cos(robotAngle)
                                - robotToShooter.getX() * Math.sin(robotAngle));
        double robotVelocityY = robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToShooter.getX() * Math.cos(robotAngle)
                                - robotToShooter.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (shooter) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = shooterPosition;
        double lookaheadToTargetDistance = shooterToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadToTargetDistance);
            double offsetX = robotVelocityX * timeOfFlight;
            double offsetY = robotVelocityY * timeOfFlight;
            lookaheadPose = new Pose2d(
                    shooterPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    shooterPosition.getRotation());
            lookaheadToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        desiredRobotAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        if (lastRobotAngle == null) {
            lastRobotAngle = desiredRobotAngle;
        }
        robotOmega = robotHeadingFilter.calculate(
                desiredRobotAngle.minus(lastRobotAngle).getRadians() / 0.02);
        lastRobotAngle = desiredRobotAngle;

        // Check if inside a box of bad
        // var flippedPose = AllianceFlipUtil.apply(estimatedPose);
        // boolean insideTowerBadBox =
        // towerBound.contains(flippedPose.getTranslation());
        // boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
        // boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
        // boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub ||
        // behindFarHub);

        latestHubParameters = new ShootingParameters(
                lookaheadToTargetDistance >= minDistance.getAsDouble()
                        && lookaheadToTargetDistance <= maxDistance.getAsDouble(),
                desiredRobotAngle.plus(Rotation2d.kPi),
                robotOmega,
                shotFlywheelSpeedMap.get(lookaheadToTargetDistance));

        // Log calculated values
        Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadToTargetDistance);

        return latestHubParameters;
    }

    private ShootingParameters getShrimpleParameters() {
        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

        // Calculate target
        Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        // Calculate parameters
        Translation2d robotToTarget = target.minus(estimatedPose.getTranslation());
        double targetDist = robotToTarget.getNorm();
        desiredRobotAngle = robotToTarget.getAngle();

        // Check if inside a box of bad
        // var flippedPose = AllianceFlipUtil.apply(estimatedPose);
        // boolean insideTowerBadBox =
        // towerBound.contains(flippedPose.getTranslation());
        // boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
        // boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
        // boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub ||
        // behindFarHub);

        latestHubParameters = new ShootingParameters(
                targetDist >= minDistance.getAsDouble()
                        && targetDist <= maxDistance.getAsDouble(),
                desiredRobotAngle.plus(Rotation2d.kPi),
                0.0,
                shotFlywheelSpeedMap.get(targetDist));

        // Log calculated values
        Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
        Logger.recordOutput("ShotCalculator/LookaheadPose", estimatedPose);
        Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", targetDist);

        return latestHubParameters;

    }

    public void clearHubParameters() {
        latestHubParameters = null;
    }

    public ShootingParameters getFerryParameters() {
        if (latestFerryParameters != null) {
            return latestFerryParameters;
        }

        latestFerryParameters = ParamPreset.SHERRY.getParams();

        return latestFerryParameters;
    }

    public void clearFerryParameters() {
        latestFerryParameters = null;
    }
}
