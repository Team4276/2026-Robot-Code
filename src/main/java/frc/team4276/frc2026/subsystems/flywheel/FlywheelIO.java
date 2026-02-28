package frc.team4276.frc2026.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double supplyCurrent = 0.0;
        public double statorCurrent = 0.0;
        public double tempCelsius = 0.0;

        public double velocityRPS = 0.0; // rpm
    }

    public default void updateInputs(FlywheelIOInputs inputs) {
    }

    public default void setRpm(double rpm) {
    }

    public default void setOpenLoop(double voltage) {
    }

    public default void setBrakeMode(boolean enable) {
    }
}
