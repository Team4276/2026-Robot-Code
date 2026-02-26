package frc.team4276.frc2026.subsystems.flywheel;

import static frc.team4276.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.team4276.frc2026.Ports;

public class FlywheelIOSpark implements FlywheelIO {
    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private final SparkMaxConfig config;

    private boolean brakeModeEnabled = false;

    public FlywheelIOSpark() {
        spark = new SparkMax(Ports.FLYWHEEL_FRONT, MotorType.kBrushless);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(80)
                .voltageCompensation(12.0)
                .inverted(false);
        config.encoder
                .inverted(false)
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        0.001,
                        0.0,
                        0.0);
        config.signals
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
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
    public void updateInputs(FlywheelIOInputs inputs) {
        ifOk(spark, new DoubleSupplier[] { spark::getAppliedOutput, spark::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(spark, spark::getOutputCurrent, (values) -> inputs.statorCurrent = values);
        ifOk(spark, spark::getMotorTemperature, (values) -> inputs.tempCelsius = values);

        ifOk(spark, encoder::getVelocity, (values) -> inputs.velocityRPS = values);
    }

    @Override
    public void setOpenLoop(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void setRpm(double rpm) {
        controller.setSetpoint(rpm / 60, ControlType.kVelocity);

        // spark.setVoltage(12*rpm/5000);
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
