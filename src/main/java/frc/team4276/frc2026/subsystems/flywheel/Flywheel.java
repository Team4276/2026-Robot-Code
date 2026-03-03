package frc.team4276.frc2026.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {
    private final LoggedTunableNumber tolerance = new LoggedTunableNumber("Flywheel/ToleranceRPM", 300);

    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelIO io;

    private double rpmSetpoint = 0.0;
    public Flywheel(FlywheelIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        Logger.recordOutput("Flywheel/RPM", getRPM());
        
    }

    public void setVelocity(double RPM){
        io.setRpm(RPM);
        rpmSetpoint = RPM;
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }

    public boolean atSetpoint(){
        return MathUtil.isNear(rpmSetpoint, inputs.velocityRPS * 60.0, tolerance.getAsDouble());
    }

    public double getRPM(){
        return inputs.velocityRPS * 60.0;
    }
}
