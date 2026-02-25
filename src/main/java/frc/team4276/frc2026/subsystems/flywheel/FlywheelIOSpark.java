package frc.team4276.frc2026.subsystems.flywheel;

import static frc.team4276.lib.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.team4276.frc2026.Ports;

public class FlywheelIOSpark implements FlywheelIO {
    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    public FlywheelIOSpark() {
        spark = new SparkMax(Ports.FLYWHEEL_FRONT, MotorType.kBrushless);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        var config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0)
                .inverted(false);
        config.encoder
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
        inputs.velocityRPS = new double[]{encoder.getVelocity(), 0};
        inputs.appliedVolts = new double[]{spark.getBusVoltage() * spark.getAppliedOutput(), 0};
    }

    @Override
    public void setOpenLoop(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void setRpm(double rpm) {
        // controller.setSetpoint(rpm, ControlType.kVelocity);

        spark.setVoltage(12*rpm/5000);        
    }

    @Override
    public void setBrakeMode(boolean enable) {

    }
}
