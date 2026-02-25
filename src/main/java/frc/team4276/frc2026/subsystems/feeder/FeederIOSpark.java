package frc.team4276.frc2026.subsystems.feeder;

import com.revrobotics.spark.SparkMax;

import static frc.team4276.lib.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.team4276.frc2026.Ports;

public class FeederIOSpark implements FeederIO {
    private final SparkMax spark;

    public FeederIOSpark() {
        spark = new SparkMax(Ports.FEEDER, MotorType.kBrushless);
        

        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0)
                .inverted(true);
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
    public void updateInputs(FeederIOInputs inputs) {
        inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
        inputs.supplyCurrentAmps = spark.getOutputCurrent();
        inputs.tempCelsius = spark.getMotorTemperature();
    }

    @Override
    public void setOpenLoop(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enable) {

    }
}
