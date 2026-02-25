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
import frc.team4276.lib.geometry.AllianceFlipUtil;
import frc.team4276.lib.geometry.GeomUtil;

import static frc.team4276.frc2026.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static ShotCalculator instance;

  private final LinearFilter robotHeadingFilter =
      LinearFilter.movingAverage((int) (0.1 / 0.02));

  private Rotation2d lastTurretAngle;
  private Rotation2d turretAngle;
  private double turretVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d robotHeading,
      double robotOmega,
      double flywheelSpeed) {}

  // Cache parameters
  private ShootingParameters latestHubParameters = null;
  private ShootingParameters latestFerryParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;

    shotFlywheelSpeedMap.put(1.34, 1000.0);
    shotFlywheelSpeedMap.put(5.60, 5000.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootingParameters getHubParameters() {
    if (latestHubParameters != null) {
      return latestHubParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getFieldVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition = estimatedPose.transformBy(GeomUtil.toTransform2d(robotToShooter));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToShooter.getY() * Math.cos(robotAngle)
                    - robotToShooter.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToShooter.getX() * Math.cos(robotAngle)
                    - robotToShooter.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    turretVelocity =
        robotHeadingFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    
    lastTurretAngle = turretAngle;
    latestHubParameters =
        new ShootingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestHubParameters;
  }

  public void clearHubParameters() {
    latestHubParameters = null;
  }

  public ShootingParameters getFerryParameters(){
    if(latestFerryParameters != null){
        return latestFerryParameters;
    }

    latestFerryParameters = ParamPreset.SHERRY.getParams();

    return latestFerryParameters;
  }

  public void clearFerryParameters(){
    latestFerryParameters = null;
  }
}
