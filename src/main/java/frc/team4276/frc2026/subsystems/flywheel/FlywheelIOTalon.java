package frc.team4276.frc2026.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.team4276.frc2026.Ports;
import frc.team4276.lib.PhoenixUtil;
import frc.team4276.lib.TalonFXFactory;
import frc.team4276.lib.TalonFXFactory.CanBus;

public class FlywheelIOTalon implements FlywheelIO {
    private final TalonFX talon;
    private final TalonFX followerTalon;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<AngularVelocity> velocity;

    private final StatusSignal<Current> supplyCurrentLeader;
    private final StatusSignal<Current> statorCurrentLeader;
    private final StatusSignal<Temperature> temperatureLeader;
    private final StatusSignal<Current> supplyCurrentFollower;
    private final StatusSignal<Current> statorCurrentFollower;
    private final StatusSignal<Temperature> temperatureFollower;

    private final VoltageOut voltageOut;
    private final VelocityVoltage velocityVoltage;

    private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

    private boolean brakeModeEnabled = false;

    public FlywheelIOTalon() {
        talon = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_RIGHT, CanBus.RIO);
        followerTalon = TalonFXFactory.createPermanentFollowerTalon(Ports.FLYWHEEL_LEFT, Ports.FLYWHEEL_RIGHT,
                CanBus.CANIVORE, true);

        // Configure motor
        var config = new TalonFXConfiguration();
        config.CurrentLimits
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true);

        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));

        config.Slot0.kP = 0.0001;

        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
        velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0);

        appliedVolts = talon.getMotorVoltage();
        supplyCurrentLeader = talon.getSupplyCurrent();
        statorCurrentLeader = talon.getStatorCurrent();
        temperatureLeader = talon.getDeviceTemp();
        supplyCurrentFollower = followerTalon.getSupplyCurrent();
        statorCurrentFollower = followerTalon.getStatorCurrent();
        temperatureFollower = followerTalon.getDeviceTemp();
        velocity = talon.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                appliedVolts,
                supplyCurrentLeader,
                statorCurrentLeader,
                temperatureLeader,
                supplyCurrentFollower,
                statorCurrentFollower,
                temperatureFollower,
                velocity);

        BaseStatusSignal.setUpdateFrequencyForAll(50, talon.getFaultField(), followerTalon.getFaultField());

        talon.optimizeBusUtilization();
        followerTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        // Update inputs
        inputs.connected[0] = leaderConnectedDebounce
                .calculate(BaseStatusSignal
                        .refreshAll(appliedVolts, supplyCurrentLeader, statorCurrentLeader, temperatureLeader)
                        .isOK());
        inputs.connected[1] = followerConnectedDebounce
                .calculate(BaseStatusSignal
                        .refreshAll(supplyCurrentFollower, statorCurrentFollower, temperatureFollower)
                        .isOK());

        BaseStatusSignal.refreshAll(
                appliedVolts,
                velocity,
                supplyCurrentLeader,
                statorCurrentLeader,
                temperatureLeader,
                supplyCurrentFollower,
                statorCurrentFollower,
                temperatureFollower);

        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.velocityRPM = velocity.getValueAsDouble();

        inputs.supplyCurrent[0] = supplyCurrentLeader.getValueAsDouble();
        inputs.statorCurrent[0] = statorCurrentLeader.getValueAsDouble();
        inputs.tempCelsius[0] = temperatureLeader.getValueAsDouble();

        inputs.supplyCurrent[1] = supplyCurrentFollower.getValueAsDouble();
        inputs.statorCurrent[1] = statorCurrentFollower.getValueAsDouble();
        inputs.tempCelsius[1] = temperatureFollower.getValueAsDouble();
    }

    @Override
    public void setOpenLoop(double voltage) {
        talon.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setRpm(double rpm) {
        setRpm(rpm, 0.0);
    }

    @Override
    public void setRpm(double rpm, double feedforward) {
        talon.setControl(velocityVoltage
                .withVelocity(rpm)
                .withFeedForward(feedforward));
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
