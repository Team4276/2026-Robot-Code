package frc.team4276.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = true;

    public double appliedVolts = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;

    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {
  }

  public default void setOpenLoop(double volts) {
  }

  public default void setBrakeMode(boolean enable) {
  }
}
