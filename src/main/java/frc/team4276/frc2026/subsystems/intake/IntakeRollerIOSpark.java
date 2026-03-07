package frc.team4276.frc2026.subsystems.intake;

import com.revrobotics.spark.SparkMax;

import static frc.team4276.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.team4276.frc2026.Ports;

public class IntakeRollerIOSpark implements IntakeRollerIO {
    private final SparkMax spark;
    private final SparkMaxConfig config;

    private boolean brakeModeEnabled = false;

    public IntakeRollerIOSpark() {
        spark = new SparkMax(Ports.INTAKE_ROLLERS, MotorType.kBrushless);

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0)
                .inverted(true)
                .openLoopRampRate(0.5)
                .closedLoopRampRate(0.5);
        config.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                spark,
                5,
                () -> spark.configure(
                        config,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters));
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        ifOk(spark, new DoubleSupplier[] { spark::getAppliedOutput, spark::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(spark, spark::getOutputCurrent, (values) -> inputs.statorCurrent = values);
        ifOk(spark, spark::getMotorTemperature, (values) -> inputs.tempCelsius = values);
    }

    @Override
    public void setOpenLoop(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;
        new Thread(
                () -> {
                    tryUntilOk(
                            spark,
                            5,
                            () -> spark.configure(
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
