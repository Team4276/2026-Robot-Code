package frc.team4276.frc2026.subsystems.drive.Module;

import static frc.team4276.frc2026.subsystems.drive.DriveConstants.*;
import static frc.team4276.lib.SparkUtil.*;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2026.subsystems.drive.PhoenixOdometryThread;
import frc.team4276.lib.PhoenixUtil;
import frc.team4276.lib.TalonFXFactory;
import frc.team4276.lib.TalonFXFactory.CanBus;

public class ModuleIOKreo implements ModuleIO {
    private final Rotation2d zeroRotation;
    private final Rotation2d zeroHelperRotation;

    // Hardware objects
    private final TalonFX driveTalon;
    private final SparkMax turnSpark;
    private final AbsoluteEncoder turnEncoder;
    private final SparkMaxConfig turnConfig;

    // Closed loop controllers
    private final SparkClosedLoopController turnController;

    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveSupplyCurrent;
    private final StatusSignal<Temperature> driveTemperature;

    private final VoltageOut driveControlSetterVoltageOut;
    private final VelocityVoltage driveControlSetterVelocityVoltage;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    private boolean brakeModeEnabled = true;

    public ModuleIOKreo(int module) {
        zeroRotation = zeroRotations[module];
        zeroHelperRotation = zeroHelperRotations[module];

        driveTalon = TalonFXFactory.createDefaultTalon(canIds[module][0], CanBus.CANIVORE);
        turnSpark = new SparkMax(canIds[module][1], MotorType.kBrushless);
        turnEncoder = turnSpark.getAbsoluteEncoder();
        turnController = turnSpark.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0
                .withKP(0.0) // TODO: tune
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(12.0 / 100.0)
                .withKA(0.0);
        driveConfig.CurrentLimits
                .withSupplyCurrentLimit(50)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(driveMotorCurrentLimit) // TODO: tune
                .withStatorCurrentLimitEnable(true);

        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig));

        // Configure turn motor
        turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig.absoluteEncoder
                .inverted(turnEncoderInverted)
                .positionConversionFactor(turnEncoderPositionFactor)
                .velocityConversionFactor(turnEncoderVelocityFactor)
                .averageDepth(2);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .pid(turnKp, 0.0, turnKd);
        turnConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        driveControlSetterVoltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
        driveControlSetterVelocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0);

        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveSupplyCurrent = driveTalon.getSupplyCurrent();
        driveTemperature = driveTalon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTemperature);

        drivePosition = driveTalon.getPosition().clone();
        driveVelocity = driveTalon.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                odometryFrequency,
                drivePosition,
                driveVelocity);

        BaseStatusSignal.setUpdateFrequencyForAll(50, driveTalon.getFaultField());

        driveTalon.optimizeBusUtilization();

        // Create odometry queues
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition);
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce
                .calculate(BaseStatusSignal.refreshAll(drivePosition, driveVelocity).isOK());

        inputs.drivePositionRad = Units.rotationsToRadians(
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(drivePosition, driveVelocity)
                        / driveMotorReduction);
        inputs.driveVelocityRadPerSec = Units
                .rotationsToRadians(driveVelocity.getValueAsDouble() / driveMotorReduction);

        BaseStatusSignal.refreshAll(driveAppliedVolts, driveSupplyCurrent, driveTemperature);

        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveSupplyCurrent.getValueAsDouble();
        inputs.driveTempCelsius = driveTemperature.getValueAsDouble();

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        ifOk(turnSpark, turnSpark::getMotorTemperature, (value) -> inputs.turnTempCelsius = value);
        inputs.zeroHelperTurnPosition = inputs.turnPosition.minus(zeroHelperRotation);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(driveControlSetterVoltageOut.withOutput(output));
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void runDriveVelocitySetpoint(double velocityRadPerSec) {
        if (DriverStation.isAutonomous()) {
            driveTalon.setControl(driveControlSetterVelocityVoltage
                    .withVelocity(Units.radiansToRotations(velocityRadPerSec * driveMotorReduction)));

        } else {
            double velocityMetresPerSec = velocityRadPerSec * wheelRadiusMeters;

            double output = (velocityMetresPerSec / maxVelocityMPS) * 12.0;

            driveTalon.setControl(driveControlSetterVoltageOut.withOutput(output));

        }
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
                rotation.plus(zeroRotation).getRadians(), 0, 2 * Math.PI);
        turnController.setSetpoint(setpoint, ControlType.kPosition);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;

        new Thread(
                () -> {
                    tryUntilOk(
                            turnSpark,
                            5,
                            () -> turnSpark.configure(
                                    turnConfig.idleMode(
                                            brakeModeEnabled
                                                    ? SparkBaseConfig.IdleMode.kBrake
                                                    : SparkBaseConfig.IdleMode.kCoast),
                                    ResetMode.kNoResetSafeParameters,
                                    PersistMode.kNoPersistParameters));

                    var configs = new MotorOutputConfigs();

                    var status = driveTalon.getConfigurator().refresh(configs);
                    if (status.isOK()) {
                        configs.NeutralMode = brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                        status = driveTalon.getConfigurator().apply(configs);
                    }
                    if (!status.isOK()) {
                        System.out.println(
                                "TalonFX ID " + driveTalon.getDeviceID() + " failed config neutral mode with error "
                                        + status.toString());
                    }
                })
                .start();
    }

}
