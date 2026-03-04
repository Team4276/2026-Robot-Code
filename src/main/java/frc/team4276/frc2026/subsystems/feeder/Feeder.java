package frc.team4276.frc2026.subsystems.feeder;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class Feeder extends SubsystemBase {
    public enum SystemState {
        IDLE(new LoggedTunableNumber("Feeder/IdleVolts", -2.0)),
        STOPPED(() -> 0.0),
        FEED(new LoggedTunableNumber("Feeder/FeedVolts", 12.0));

        private final DoubleSupplier voltage;

        SystemState(DoubleSupplier voltage) {
            this.voltage = voltage;
        }

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    private SystemState systemState = SystemState.IDLE;

    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
    private final FeederIO io;

    private double directionFactor = 1.0;
    private final Timer directionSwap = new Timer();
    private final LoggedTunableNumber directionSwapTime = new LoggedTunableNumber("Feeder/DirectionSwapTime", 0.5);

    public Feeder(FeederIO io) {
        this.io = io;

        directionSwap.restart();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        double outputVoltage = systemState.getVoltage();

        if(directionSwap.get() > directionSwapTime.getAsDouble()){
            directionFactor *= -1.0;
            directionSwap.restart();
        }

        if(systemState == SystemState.IDLE){
            outputVoltage *= directionFactor;
        }

        io.setOpenLoop(outputVoltage);

        Logger.recordOutput("Feeder/SystemState", systemState);
    }

    public void setSystemState(SystemState state) {
        systemState = state;
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
}
