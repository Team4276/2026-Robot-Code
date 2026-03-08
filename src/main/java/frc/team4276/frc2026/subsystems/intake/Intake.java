package frc.team4276.frc2026.subsystems.intake;

import static frc.team4276.frc2026.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private final IntakeDeployIO deployIo;
    private final IntakeRollerIO rollerIo;
    private final IntakeDeployIOInputsAutoLogged deployInputs = new IntakeDeployIOInputsAutoLogged();
    private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

    private final LoggedTunableNumber absoluteEncoderZero = new LoggedTunableNumber("Intake/AbsoluteEncoderZero", 0.24);

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

    private boolean isDisabled = false;

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public Intake(IntakeDeployIO deployIo, IntakeRollerIO rollerIo) {
        this.deployIo = deployIo;
        this.rollerIo = rollerIo;
    }

    @Override
    public void periodic() {
        deployIo.updateInputs(deployInputs);
        rollerIo.updateInputs(rollerInputs);
        Logger.processInputs("Intake/Deploy", deployInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);

        systemState = handleStateTransition();
        applyState();

        if(isDisabled && DriverStation.isEnabled()){
            deployIo.setPosition(MathUtil.inputModulus(deployInputs.absolutePositionRad - absoluteEncoderZero.getAsDouble(), -1.0, 1.0) / motorToEncoderReduction);
        }

        isDisabled = DriverStation.isDisabled();

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
                rollerIo.setOpenLoop(idleVolts.getAsDouble());
                // deployIo.setOpenLoop(deployIdleVolts.getAsDouble());

                break;

            case RETRACTED:
                rollerIo.setOpenLoop(idleVolts.getAsDouble());
                // deployIo.setPositionSetpoint(retractPosition.getAsDouble());

                break;

            case INTAKING:
                rollerIo.setOpenLoop(intakeVolts.getAsDouble());
                // deployIo.setPositionSetpoint(deployPosition.getAsDouble());

                break;
            case EXHAUSTING:
                rollerIo.setOpenLoop(exhaustVolts.getAsDouble());
                // deployIo.setPositionSetpoint(deployPosition.getAsDouble());

                break;
        }
    }

    public void setDeployVoltage(double voltage){
        deployIo.setOpenLoop(voltage);
    }

    public void setDeployed(boolean deployed){
        if(deployed){
            deployIo.setPosition(deployPosition.getAsDouble());
        } else {
            deployIo.setPosition(retractPosition.getAsDouble());
        }
    }

    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    public void setBrakeMode(boolean enabled) {
        deployIo.setBrakeMode(enabled);
    }
}
