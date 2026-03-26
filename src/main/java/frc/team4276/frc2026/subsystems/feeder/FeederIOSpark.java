package frc.team4276.frc2026.subsystems.feeder;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;

import static frc.team4276.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.team4276.frc2026.Ports;

public class FeederIOSpark implements FeederIO {
    private final SparkBase leadingSpark;
    private final SparkBase feedingSpark;
    private final SparkBaseConfig config;

    private boolean brakeModeEnabled = false;

    public FeederIOSpark() {
        leadingSpark = new SparkFlex(Ports.FEEDER_LEADER, MotorType.kBrushless);
        feedingSpark = new SparkFlex(Ports.FEEDER_FEEDER, MotorType.kBrushless);

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(80)
                .voltageCompensation(12.0)
                .inverted(true)
                .openLoopRampRate(0.5)
                .closedLoopRampRate(0.5);
        config.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                leadingSpark,
                5,
                () -> leadingSpark.configure(
                        config,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters));
        tryUntilOk(
                feedingSpark,
                5,
                () -> feedingSpark.configure(
                        config,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        ifOk(leadingSpark, new DoubleSupplier[] { leadingSpark::getAppliedOutput, leadingSpark::getBusVoltage },
                (values) -> inputs.appliedVolts[0] = values[0] * values[1]);
        ifOk(leadingSpark, leadingSpark::getOutputCurrent, (values) -> inputs.statorCurrent[0] = values);
        ifOk(leadingSpark, leadingSpark::getMotorTemperature, (values) -> inputs.tempCelsius[0] = values);

        ifOk(feedingSpark, new DoubleSupplier[] { feedingSpark::getAppliedOutput, feedingSpark::getBusVoltage },
                (values) -> inputs.appliedVolts[1] = values[0] * values[1]);
        ifOk(feedingSpark, feedingSpark::getOutputCurrent, (values) -> inputs.statorCurrent[1] = values);
        ifOk(feedingSpark, feedingSpark::getMotorTemperature, (values) -> inputs.tempCelsius[1] = values);
    }

    @Override
    public void setOpenLoop(double voltage) {
        setOpenLoop(voltage, voltage);
    }

    @Override
    public void setOpenLoop(double leaderVoltage, double feederVoltage) {
        leadingSpark.setVoltage(leaderVoltage);
        feedingSpark.setVoltage(feederVoltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;
        new Thread(
                () -> {
                    tryUntilOk(
                            leadingSpark,
                            5,
                            () -> leadingSpark.configure(
                                    config.idleMode(
                                            brakeModeEnabled
                                                    ? SparkBaseConfig.IdleMode.kBrake
                                                    : SparkBaseConfig.IdleMode.kCoast),
                                    ResetMode.kNoResetSafeParameters,
                                    PersistMode.kNoPersistParameters));
                    tryUntilOk(
                            feedingSpark,
                            5,
                            () -> feedingSpark.configure(
                                    config.idleMode(
                                            brakeModeEnabled
                                                    ? SparkBaseConfig.IdleMode.kBrake
                                                    : SparkBaseConfig.IdleMode.kCoast),
                                    ResetMode.kNoResetSafeParameters,
                                    PersistMode.kNoPersistParameters));
                })
                .start();
    }
}
