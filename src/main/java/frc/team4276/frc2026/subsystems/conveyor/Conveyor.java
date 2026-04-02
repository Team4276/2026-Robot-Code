package frc.team4276.frc2026.subsystems.conveyor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class Conveyor extends SubsystemBase {
    public enum SystemState {
        IDLE(new LoggedTunableNumber("Conveyor/IdleVolts", 0.0)),
        STOPPED(() -> 0.0),
        SPINUP(new LoggedTunableNumber("Conveyor/SpinupVolts", -12.0)),
        FEED(new LoggedTunableNumber("Conveyor/FeedVolts", 12.0)),
        EXHAUST(new LoggedTunableNumber("Conveyor/ExhaustVolts", -12.0));

        private final DoubleSupplier voltage;

        SystemState(DoubleSupplier voltage) {
            this.voltage = voltage;
        }

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    private SystemState systemState = SystemState.IDLE;

    private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();
    private final ConveyorIO io;

    private double directionFactor = 1.0;
    private final Timer directionSwap = new Timer();
    private final LoggedTunableNumber directionSwapTime = new LoggedTunableNumber("Conveyor/DirectionSwapTime", 0.25);

    public Conveyor(ConveyorIO io) {
        this.io = io;

        directionSwap.restart();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Conveyor", inputs);

        double outputVoltage = systemState.getVoltage();

        if (directionSwap.get() > directionSwapTime.getAsDouble()) {
            directionFactor *= -1.0;
            directionSwap.restart();
        }

        if (systemState == SystemState.IDLE) {
            // outputVoltage *= directionFactor;
        }

        io.setOpenLoop(outputVoltage);

        Logger.recordOutput("Conveyor/SystemState", systemState);
    }

    public void setSystemState(SystemState state) {
        systemState = state;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
}
