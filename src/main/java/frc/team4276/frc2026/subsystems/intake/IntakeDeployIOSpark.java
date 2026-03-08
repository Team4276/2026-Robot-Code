package frc.team4276.frc2026.subsystems.intake;

import static frc.team4276.lib.SparkUtil.*;
import static frc.team4276.frc2026.subsystems.intake.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.team4276.frc2026.Ports;

public class IntakeDeployIOSpark implements IntakeDeployIO {
    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController controller;

    private final SparkMaxConfig config;

    private boolean brakeModeEnabled = false;

    public IntakeDeployIOSpark() {
        spark = new SparkMax(Ports.INTAKE_DEPLOY, MotorType.kBrushless);
        encoder = spark.getEncoder();
        absoluteEncoder = spark.getAbsoluteEncoder();
        controller = spark.getClosedLoopController();

        config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0)
                .inverted(true);
        config.encoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        config.absoluteEncoder
                .inverted(false)
                .positionConversionFactor(2 * Math.PI)
                .velocityConversionFactor(2 * Math.PI / 60)
                // .positionConversionFactor(1.0)
                // .velocityConversionFactor(1.0)
                .averageDepth(2)
                // .zeroOffset(0.0)
                ;
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        0.0,
                        0.0,
                        0.0);
        config.closedLoop.feedForward
                .kS(0.0)
                // .kG(0.0)
                .kCos(0.0)
                .kCosRatio(motorReduction)
                .kV(12.0 / 5676.0);
        config.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .allowedProfileError(0.0)
                .cruiseVelocity(0.0)
                .maxAcceleration(0.0);
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
    public void updateInputs(IntakeDeployIOInputs inputs) {
        ifOk(spark, new DoubleSupplier[] { spark::getAppliedOutput, spark::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(spark, spark::getOutputCurrent, (values) -> inputs.statorCurrent = values);
        ifOk(spark, spark::getMotorTemperature, (values) -> inputs.tempCelsius = values);

        ifOk(spark, encoder::getPosition, (values) -> inputs.positionRev = values);
        ifOk(spark, absoluteEncoder::getPosition, (values) -> inputs.absolutePositionRad = values);
    }

    @Override
    public void setOpenLoop(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void setPositionSetpoint(double position) {
        controller.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
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
