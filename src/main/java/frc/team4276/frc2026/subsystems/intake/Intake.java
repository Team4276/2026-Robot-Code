package frc.team4276.frc2026.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum WantedState {
        IDLE,
        INTAKE,
        EXHAUST
    }

    public enum SystemState {
        IDLING(0.0),
        INTAKING(10.0),
        EXHAUSTING(-5.0);

        final double volts;

        SystemState(double volts){
            this.volts = volts;
        }
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public Intake(IntakeIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        systemState = handleStateTransition();
        applyState();

        Logger.recordOutput("Intake/SystemState", systemState);
        Logger.recordOutput("Intake/DesiredState", wantedState);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case INTAKE -> SystemState.INTAKING;
            case EXHAUST -> SystemState.EXHAUSTING;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                io.setVoltage(systemState.volts);

                break;
            case INTAKING:
                io.setVoltage(systemState.volts);

                break;
            case EXHAUSTING:
                io.setVoltage(systemState.volts);

                break;
        }
    }
}
