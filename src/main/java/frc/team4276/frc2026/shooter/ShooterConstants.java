package frc.team4276.frc2026.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;
import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class ShooterConstants {
    public static final Transform3d robotToShooter = new Transform3d(Units.inchesToMeters(-7.5), 0.0, 0.0, new Rotation3d());

    public static enum ParamPreset {
        // Shooting Presets
        SHOWER(Rotation2d.kZero,
                0.0,
                new LoggedTunableNumber("Shooter/Presets/ShowerRPM", 3000.0)),
        SHUB(Rotation2d.kZero,
                0.0,
                new LoggedTunableNumber("Shooter/Presets/ShubRPM",2750.0)),
        SHERRY(Rotation2d.kZero,
                0.0,
                new LoggedTunableNumber("Shooter/Presets/SherryRPM", 5000.0)),

        // Other
        STOW(Rotation2d.kZero,
                0.0,
                new LoggedTunableNumber("Shooter/Presets/StowRPM", 300.0)),
        TURTLE(Rotation2d.kZero,
                0.0,
                new LoggedTunableNumber("Shooter/Presets/TurtleRPM", 300.0));

        private final Rotation2d robotHeading;
        private final double robotOmega;
        private final DoubleSupplier flywheelSpeed;

        ParamPreset(Rotation2d robotHeading, double robotOmega, DoubleSupplier flywheelSpeed) {
            this.robotHeading = robotHeading;
            this.robotOmega = robotOmega;
            this.flywheelSpeed = flywheelSpeed;
        }

        public ShootingParameters getParams() {
            return new ShootingParameters(true, robotHeading, robotOmega, flywheelSpeed.getAsDouble());
        }
    }
}
