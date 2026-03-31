package frc.team4276.frc2026.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
    @AutoLog
    public static class ConveyorIOInputs {
        public boolean[] connected = {true, true};

        public double[] appliedVolts = {0.0, 0.0};
        public double[] supplyCurrent = {0.0, 0.0};
        public double[] statorCurrent = {0.0, 0.0};

        public double[] tempCelsius = {0.0, 0.0};
    }

    public default void updateInputs(ConveyorIOInputs inputs) {
    }

    public default void setOpenLoop(double voltage) {
    }

    public default void setOpenLoop(double leaderVoltage, double followerVoltage) {
    }

    public default void setBrakeMode(boolean enable) {
    }
}
