package frc.team4276.frc2026.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.team4276.frc2026.Ports;
import frc.team4276.lib.PhoenixUtil;
import frc.team4276.lib.TalonFXFactory;
import frc.team4276.lib.TalonFXFactory.CanBus;

public class IntakeRollerIOTalon implements IntakeRollerIO {
    private final TalonFX talon;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;

    private final VoltageOut voltageOut;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    private boolean brakeModeEnabled = false;

    public IntakeRollerIOTalon() {
        talon = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLERS, CanBus.CANIVORE);

        // Configure motor
        var config = new TalonFXConfiguration();
        config.CurrentLimits
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true);

        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));

        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);

        appliedVolts = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        temperature = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                appliedVolts,
                supplyCurrent,
                temperature);

        BaseStatusSignal.setUpdateFrequencyForAll(50, talon.getFaultField());

        talon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        // Update inputs
        inputs.connected = connectedDebounce
                .calculate(BaseStatusSignal.refreshAll(appliedVolts, supplyCurrent, temperature)
                        .isOK());

        BaseStatusSignal.refreshAll(appliedVolts, supplyCurrent, temperature);

        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.tempCelsius = temperature.getValueAsDouble();
    }

    @Override
    public void setOpenLoop(double voltage) {
        talon.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // if (brakeModeEnabled == enabled)
        // return;
        // brakeModeEnabled = enabled;
        // new Thread(
        // () -> {
        // tryUntilOk(
        // spark,
        // 5,
        // () -> spark.configure(
        // config.idleMode(
        // brakeModeEnabled
        // ? SparkBaseConfig.IdleMode.kBrake
        // : SparkBaseConfig.IdleMode.kCoast),
        // ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters));
        // })
        // .start();
    }
}
