package frc.team4276.frc2026.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;

    public Turret(TurretIO io){
        this.io = io;
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        
    }

    public void setPositionVelocity(double position, double velocity){
        io.setPositionVelocity(position, velocity);
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }
}
