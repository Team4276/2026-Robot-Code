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

    private LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.0);
    private LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 12.0 / 5676.0);

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
        double velocityFeedforward = kV.getAsDouble() * RPM;
        
        io.setRpm(RPM, kS.getAsDouble() + velocityFeedforward);
        rpmSetpoint = RPM;
        
        Logger.recordOutput("Flywheel/VelocityFeedforward", velocityFeedforward);
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }

    public boolean atSetpoint(){
        return MathUtil.isNear(rpmSetpoint, inputs.velocityRPM, tolerance.getAsDouble());
    }

    public double getRPM(){
        return inputs.velocityRPM;
    }
}
