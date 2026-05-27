package frc.team4276.frc2026.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team4276.frc2026.Constants;
import frc.team4276.frc2026.FieldConstants.FieldZone;
import frc.team4276.frc2026.RobotState;
import frc.team4276.frc2026.shooter.ShotCalculator;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;
import frc.team4276.frc2026.subsystems.conveyor.Conveyor;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.drive.Drive.DriveSpeedScalar;
import frc.team4276.frc2026.subsystems.drive.Drive.WantedState;
import frc.team4276.frc2026.subsystems.feeder.Feeder;
import frc.team4276.frc2026.subsystems.flywheel.Flywheel;
import frc.team4276.frc2026.subsystems.intake.Intake;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.lib.dashboard.LoggedTunableNumber;
import frc.team4276.lib.hid.ViXController;

import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    private final Drive drive;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Feeder feeder;
    private final Flywheel flywheel;

    @SuppressWarnings("unused")
    private final Vision vision;

    private final ViXController controller;
    private final ViXController operator;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private enum FeedState {
        IDLE,
        FERRY,
        ACTIVE,
        FIRING,
        EXHAUST
    }

    private FeedState feedState = FeedState.IDLE;

    private Supplier<ShootingParameters> shootingParams = ParamPreset.STOW::getParams;
    private ParamPreset currPreset = ParamPreset.STOW;

    private boolean isFirstActive = false;
    private boolean isManual = false;

    private final Debouncer inShootingToleranceDebounce = new Debouncer(0.25);
    private final LoggedTunableNumber hubPrefireTime = new LoggedTunableNumber("Superstructure/HubPrefireTime", 1.0);

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public Superstructure(
            Drive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Flywheel flywheel,
            Vision vision,
            ViXController controller,
            ViXController operator) {
        this.drive = drive;
        this.intake = intake;
        this.conveyor = conveyor;
        this.feeder = feeder;
        this.flywheel = flywheel;
        this.vision = vision;
        this.controller = controller;
        this.operator = operator;

        // Rumble on hub active transitions
        new Trigger(this::isHubActive)
                .onTrue(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 0.25, 3))
                .onFalse(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 1.0, 1));

        // Flywheel manual override binding
        new Trigger(() -> operator.a().getAsBoolean())
                .onTrue(Commands.runOnce(() -> flywheel.setVoltage(-12.0)))
                .onFalse(Commands.runOnce(() -> flywheel.setVelocity(shootingParams.get().flywheelSpeed())));
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        ShotCalculator.getInstance().clearHubParameters();
        ShotCalculator.getInstance().clearFerryParameters();

        if (DriverStation.isDisabled()) {
            drive.setWantedState(WantedState.TELEOP);
            currPreset = ParamPreset.TURTLE;
            shootingParams = currPreset::getParams;
            feedState = FeedState.IDLE;
        }

        if (Constants.isTuning) {
            ShotCalculator.getInstance().getHubParameters();
        }

        updateFeedState();
        applyFeedState();
        updateLogging();
    }

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------

    private void updateFeedState() {
        // ACTIVE transitions to FIRING once shooter is at setpoint and conditions met
        if (feedState == FeedState.ACTIVE
                && inShootingToleranceDebounce.calculate(shooterAtSetpoint())
                && (shouldShootHub() || isManual)) {
            feedState = FeedState.FIRING;
        }

        // FERRY transitions to FIRING once shooter is at setpoint
        if (feedState == FeedState.FERRY
                && inShootingToleranceDebounce.calculate(shooterAtSetpoint())) {
            feedState = FeedState.FIRING;
        }
    }

    private void applyFeedState() {
        switch (feedState) {
            case IDLE:
                flywheel.setVelocity(shootingParams.get().flywheelSpeed());
                feeder.setSystemState(Feeder.SystemState.IDLE);
                conveyor.setSystemState(Conveyor.SystemState.IDLE);
                break;

            case ACTIVE:
            case FERRY:
                flywheel.setVelocity(shootingParams.get().flywheelSpeed());
                feeder.setSystemState(Feeder.SystemState.SPINUP);
                conveyor.setSystemState(Conveyor.SystemState.SPINUP);
                break;

            case FIRING:
                flywheel.setVelocity(shootingParams.get().flywheelSpeed());
                feeder.setSystemState(Feeder.SystemState.FEED);
                conveyor.setSystemState(Conveyor.SystemState.FEED);
                break;

            case EXHAUST:
                flywheel.setVelocity(shootingParams.get().flywheelSpeed());
                feeder.setSystemState(Feeder.SystemState.EXHAUST);
                conveyor.setSystemState(Conveyor.SystemState.EXHAUST);
                break;
        }
    }

    private void updateLogging() {
        Logger.recordOutput("Superstructure/FeedState", feedState);
        Logger.recordOutput("Superstructure/IsFirstActive", isFirstActive);
        Logger.recordOutput("Superstructure/IsHubActive", isHubActive());
        Logger.recordOutput("Superstructure/ShouldShootHub", shouldShootHub());
        Logger.recordOutput("Superstructure/ShooterAtSetpoint", shooterAtSetpoint());
        Logger.recordOutput("Superstructure/ParamPreset", currPreset);
        Logger.recordOutput("Superstructure/PeriodCountdown", getPeriodCountDown());
        Logger.recordOutput("Superstructure/MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("Superstructure/PeriodName", getCurrentPeriod());
        Logger.recordOutput("Superstructure/IsManual", isManual);
    }

    // -------------------------------------------------------------------------
    // Commands
    // -------------------------------------------------------------------------

    public Command deployIntake() {
        return Commands.runOnce(() -> intake.setRollerState(Intake.RollerState.INTAKING));
    }

    public Command retractIntake() {
        return Commands.runOnce(() -> intake.setRollerState(Intake.RollerState.IDLE));
    }

    public Command enableShooter() {
        return Commands.runOnce(() -> {
            if (RobotState.getInstance().getCurrentFieldZone() == FieldZone.ALLIANCE) {
                shootingParams = ShotCalculator.getInstance()::getHubParameters;
                drive.setVelocityScalar(DriveSpeedScalar.CRAWL);
                drive.setHeadingAlignRotation(() -> shootingParams.get().robotHeading());
                feedState = FeedState.ACTIVE;
            } else {
                shootingParams = ShotCalculator.getInstance()::getFerryParameters;
                drive.setWantedState(WantedState.TELEOP);
                feedState = FeedState.FERRY;
            }
        });
    }

    public Command disableShooter() {
        return Commands.runOnce(() -> {
            shootingParams = ParamPreset.STOW::getParams;
            currPreset = ParamPreset.STOW;
            drive.setVelocityScalar(DriveSpeedScalar.DEFAULT);
            drive.setWantedState(WantedState.TELEOP);
            feedState = FeedState.IDLE;
        });
    }

    public Command shootPreset(ParamPreset preset) {
        return Commands.runOnce(() -> {
            currPreset = preset;
            shootingParams = currPreset::getParams;
            drive.setWantedState(WantedState.TELEOP);

            if (preset == ParamPreset.SHOWER || preset == ParamPreset.SHUB) {
                feedState = FeedState.ACTIVE;
            } else if (preset == ParamPreset.SHERRY) {
                feedState = FeedState.FERRY;
            }
        });
    }

    public Command turtle() {
        return Commands.runOnce(() -> {
            intake.setRollerState(Intake.RollerState.EXHAUSTING);
            drive.setWantedState(WantedState.TELEOP);
            currPreset = ParamPreset.TURTLE;
            shootingParams = currPreset::getParams;
            feedState = FeedState.EXHAUST;
        });
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    public void setIsManual(boolean isManual) {
        this.isManual = isManual;
    }

    public boolean getIsManual() {
        return isManual;
    }

    public void setIsFirstActive(boolean isFirstActive) {
        this.isFirstActive = isFirstActive;
    }

    public boolean getIsFirstActive() {
        return isFirstActive;
    }

    public boolean shooterAtSetpoint() {
        return Constants.isSim ||
                (flywheel.atSetpoint()
                        && (drive.getSystemState() == Drive.SystemState.HEADING_ALIGN
                                ? drive.isAtHeading()
                                : true));
    }

    // -------------------------------------------------------------------------
    // Match period logic
    // -------------------------------------------------------------------------

    private enum MatchPeriod {
        AUTO, TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, ENDGAME;

        static MatchPeriod fromMatchTime(double t, boolean isAuto) {
            if (isAuto)    return AUTO;
            if (t > 130)   return TRANSITION;
            if (t > 105)   return SHIFT_1;
            if (t > 80)    return SHIFT_2;
            if (t > 55)    return SHIFT_3;
            if (t > 30)    return SHIFT_4;
            return ENDGAME;
        }

        double countdown(double t) {
            return switch (this) {
                case AUTO       -> t;
                case TRANSITION -> t - 130;
                case SHIFT_1    -> t - 105;
                case SHIFT_2    -> t - 80;
                case SHIFT_3    -> t - 55;
                case SHIFT_4    -> t - 30;
                case ENDGAME    -> t;
            };
        }

        String label() {
            return switch (this) {
                case AUTO       -> "Auto";
                case TRANSITION -> "Transition";
                case SHIFT_1    -> "Shift 1";
                case SHIFT_2    -> "Shift 2";
                case SHIFT_3    -> "Shift 3";
                case SHIFT_4    -> "Shift 4";
                case ENDGAME    -> "Endgame";
            };
        }
    }

    private MatchPeriod getCurrentMatchPeriod() {
        return MatchPeriod.fromMatchTime(DriverStation.getMatchTime(), DriverStation.isAutonomous());
    }

    public boolean isHubActive() {
        double matchTime = DriverStation.getMatchTime();
        MatchPeriod period = getCurrentMatchPeriod();

        if (period == MatchPeriod.AUTO
                || period == MatchPeriod.ENDGAME
                || period == MatchPeriod.TRANSITION) {
            return true;
        }

        boolean firstActiveShift = (period == MatchPeriod.SHIFT_1 || period == MatchPeriod.SHIFT_3);
        return firstActiveShift ? isFirstActive : !isFirstActive;
    }

    public boolean shouldShootHub() {
        if (!isHubActive()) {
            return getPeriodCountDown() < hubPrefireTime.getAsDouble();
        }
        return true;
    }

    public double getPeriodCountDown() {
        double t = DriverStation.getMatchTime();
        return getCurrentMatchPeriod().countdown(t);
    }

    public String getCurrentPeriod() {
        return getCurrentMatchPeriod().label();
    }
}