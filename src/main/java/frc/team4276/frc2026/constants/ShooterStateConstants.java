package frc.team4276.frc2026.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterStateConstants {
    public static record ShooterState(double rpm) {
    }

    private static final InterpolatingDoubleTreeMap flywheelHubMap = new InterpolatingDoubleTreeMap(); // RPM
    static {
        flywheelHubMap.put(1.0, 1000.0);
        flywheelHubMap.put(10.0, 5000.0);
    }

    public static ShooterState getHubShooterStateFromDistance(double distance){
        return new ShooterState(flywheelHubMap.get(distance));
    }

    private static final InterpolatingDoubleTreeMap flywheelFerryMap = new InterpolatingDoubleTreeMap(); // RPM
    static {
        flywheelFerryMap.put(1.0, 1000.0);
        flywheelFerryMap.put(10.0, 5000.0);
    }

    public static ShooterState getFerryShooterStateFromDistance(double distance){
        return new ShooterState(flywheelHubMap.get(distance));
    }

    // Passing?
}
