package frc.team4276.frc2026.subsystems.intake;

import static frc.team4276.frc2026.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeDeployIO deployIo;
    private IntakeDeployIOInputsAutoLogged deployInputs = new IntakeDeployIOInputsAutoLogged();

    public enum WantedState {
        IDLE,
        RETRACT,
        INTAKE,
        EXHAUST
    }

    public enum SystemState {
        IDLING,
        RETRACTED,
        INTAKING,
        EXHAUSTING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public Intake(IntakeDeployIO deployIo){
        this.deployIo = deployIo;
    }

    @Override
    public void periodic() {
        deployIo.updateInputs(deployInputs);
        Logger.processInputs("Intake/Deploy", deployInputs);

        systemState = handleStateTransition();
        applyState();

        Logger.recordOutput("Intake/SystemState", systemState);
        Logger.recordOutput("Intake/DesiredState", wantedState);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case RETRACT -> SystemState.RETRACTED;
            case INTAKE -> SystemState.INTAKING;
            case EXHAUST -> SystemState.EXHAUSTING;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                deployIo.setOpenLoop(idleVolts);

                break;

            case RETRACTED:
                deployIo.setOpenLoop(idleVolts);
                deployIo.setPosition(retractPosition);

                break;
            
            case INTAKING:
                deployIo.setOpenLoop(intakeVolts);
                deployIo.setPosition(deployPosition);

                break;
            case EXHAUSTING:
                deployIo.setOpenLoop(exhaustVolts);
                deployIo.setPosition(deployPosition);

                break;
        }
    }

    public void setWantedState(WantedState state){
        wantedState = state;
    }
    
    public void setBrakeMode(boolean enabled){
        deployIo.setBrakeMode(enabled);
    }
}
