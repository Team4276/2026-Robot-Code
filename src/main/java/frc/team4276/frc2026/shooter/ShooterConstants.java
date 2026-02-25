package frc.team4276.frc2026.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;

public class ShooterConstants {
    public static final double tolerance = 10; // rpm

    public static final Transform3d robotToTurret = Transform3d.kZero;

    public static enum ParamPreset {
        // Shooting Presets
        SHOWER(Rotation2d.kZero,
                        0.0,
                        3000.0),
        SHUB(Rotation2d.kZero,
                        0.0,
                        3000.0),
        SHERRY(Rotation2d.kZero,
                        0.0,
                        3000.0),
        SHTEAL(Rotation2d.kZero,
                        0.0,
                        3000.0),
                        
        // Other
        STOW(Rotation2d.kZero,
                        0.0,
                        0.0),
        TURTLE(Rotation2d.kZero,
                        0.0,
                        0.0);

        private final ShootingParameters params;

        ParamPreset(Rotation2d robotHeading, double robotOmega, double flywheelSpeed){
            this.params = new ShootingParameters(true, robotHeading, robotOmega, flywheelSpeed);
        }

        public ShootingParameters getParams(){
            return params;
        }
    }
}
