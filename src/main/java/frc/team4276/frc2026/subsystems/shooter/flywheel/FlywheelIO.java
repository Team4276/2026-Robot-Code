package frc.team4276.frc2026.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = true;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public double velocity = 0.0; // rpm
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setRpm(double rpm){}
}
