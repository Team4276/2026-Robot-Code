package frc.team4276.frc2026.subsystems.intake;

import static frc.team4276.frc2026.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private final IntakeDeployIO deployIo;
    private final IntakeRollerIO rollerIo;
    private final IntakeDeployIOInputsAutoLogged deployInputs = new IntakeDeployIOInputsAutoLogged();
    private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

    private final LoggedTunableNumber absoluteEncoderZero = new LoggedTunableNumber("Intake/AbsoluteEncoderZero", 1.515);

    public enum DeployState {
        IDLE,
        MANUAL
    }

    public enum RollerState {
        IDLE,
        INTAKING,
        EXHAUSTING
    }

    private DeployState deployState = DeployState.IDLE;
    private RollerState rollerState = RollerState.IDLE;

    private double manualDeployVoltage = 0.0;

    private boolean isDisabled = false;

    private final Debouncer currentDebounce = new Debouncer(0.25, DebounceType.kRising);

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

        // Re-zero encoder on enable
        if (isDisabled && DriverStation.isEnabled()) {
            deployIo.setPosition(
                MathUtil.inputModulus(
                    deployInputs.absolutePositionRad - absoluteEncoderZero.getAsDouble(),
                    -1.0, 1.0)
                / motorToEncoderReduction);
        }
        isDisabled = DriverStation.isDisabled();

        applyDeployState();
        applyRollerState();

        Logger.recordOutput("Intake/DeployState", deployState);
        Logger.recordOutput("Intake/RollerState", rollerState);
        Logger.recordOutput("Intake/ManualDeployVoltage", manualDeployVoltage);
    }

    // -------------------------------------------------------------------------
    // Deploy
    // -------------------------------------------------------------------------

    private void applyDeployState() {
        switch (deployState) {
            case IDLE:
                deployIo.setOpenLoop(0.0);
                break;

            case MANUAL:
                deployIo.setOpenLoop(manualDeployVoltage);
                break;
        }
    }

    /**
     * Set open-loop voltage for the deploy arm.
     * Pass 0.0 to return to IDLE (motors off).
     */
    public void setManualDeploy(double voltage) {
        manualDeployVoltage = voltage;
        deployState = (voltage != 0.0) ? DeployState.MANUAL : DeployState.IDLE;
    }

    // -------------------------------------------------------------------------
    // Roller
    // -------------------------------------------------------------------------

    private void applyRollerState() {
        switch (rollerState) {
            case IDLE:
                rollerIo.setOpenLoop(idleVolts.getAsDouble());
                break;

            case INTAKING:
                rollerIo.setOpenLoop(intakeVolts.getAsDouble());
                break;

            case EXHAUSTING:
                rollerIo.setOpenLoop(exhaustVolts.getAsDouble());
                break;
        }
    }

    public void setRollerState(RollerState state) {
        rollerState = state;
    }

    // -------------------------------------------------------------------------
    // Shared utilities
    // -------------------------------------------------------------------------

    /** True when the roller is drawing enough current to indicate a stall (note held). */
    public boolean isStalling() {
        return currentDebounce.calculate(rollerInputs.statorCurrent >= 40.0);
    }

    public void setBrakeMode(boolean enabled) {
        deployIo.setBrakeMode(enabled);
    }
}