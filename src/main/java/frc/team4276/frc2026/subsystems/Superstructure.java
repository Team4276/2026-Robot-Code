package frc.team4276.frc2026.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team4276.frc2026.Constants;
import frc.team4276.frc2026.FieldConstants;
import frc.team4276.frc2026.RobotState;
import frc.team4276.frc2026.FieldConstants.FieldZone;
import frc.team4276.frc2026.shooter.ShotCalculator;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.drive.Drive.DriveSpeedScalar;
import frc.team4276.frc2026.subsystems.drive.Drive.WantedState;
import frc.team4276.frc2026.subsystems.feeder.Feeder;
import frc.team4276.frc2026.subsystems.flywheel.Flywheel;
import frc.team4276.frc2026.subsystems.intake.Intake;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.lib.dashboard.LoggedTunableNumber;
import frc.team4276.lib.hid.ViXController;

public class Superstructure extends SubsystemBase {
    private final Drive drive;
    private final Intake intake;
    private final Feeder feeder;
    private final Flywheel flywheel;

    @SuppressWarnings("unused")
    private final Vision vision;

    private final ViXController controller;
    private final ViXController operator;

    private boolean isFirstActive = false;

    private Supplier<ShootingParameters> shootingParams = ParamPreset.STOW::getParams;
    private ParamPreset currPreset = ParamPreset.STOW;

    private enum FeedState {
        NO,
        FERRY,
        ACTIVE
    }

    private FeedState feedState = FeedState.NO;

    private Trigger activeRumble = new Trigger(this::isHubActive);

    private boolean isManual = false;
    private Debouncer inShootingToleranceDebounce = new Debouncer(0.25);

    private final LoggedTunableNumber hubPrefireTime = new LoggedTunableNumber("Superstructure/HubPrefireTime", 1.0);

    public Superstructure(
            Drive drive,
            Intake intake,
            Feeder feeder,
            Flywheel flywheel,
            Vision vision,
            ViXController controller,
            ViXController operator) {
        this.drive = drive;
        this.intake = intake;
        this.feeder = feeder;
        this.flywheel = flywheel;
        this.vision = vision;
        this.controller = controller;
        this.operator = operator;

        activeRumble
                .onTrue(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 0.25, 3))
                .onFalse(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 1.0, 1));

        SmartDashboard.putBoolean("Superstructure/IsManual", isManual);
        SmartDashboard.putBoolean("Superstructure/IsFirstActive", isFirstActive);
    }

    @Override
    public void periodic() {
        ShotCalculator.getInstance().clearHubParameters();
        ShotCalculator.getInstance().clearFerryParameters();

        if (DriverStation.isDisabled()) {
            drive.setWantedState(WantedState.TELEOP);
            currPreset = ParamPreset.TURTLE;
            shootingParams = currPreset::getParams;
            feedState = FeedState.NO;
        }

        if (inShootingToleranceDebounce.calculate(
                shooterAtSetpoint())) {

            if (feedState == FeedState.ACTIVE && (shouldShootHub() || getIsManual())) {
                feeder.setSystemState(Feeder.SystemState.FEED);

            } else if (feedState == FeedState.FERRY) {
                feeder.setSystemState(Feeder.SystemState.FEED);

            } else {
                feeder.setSystemState(Feeder.SystemState.IDLE);

            }

        } else {
            feeder.setSystemState(Feeder.SystemState.IDLE);

        }

        if (Constants.isTuning) {
            ShotCalculator.getInstance().getHubParameters();
        }

        if (operator.a().getAsBoolean()) {
            flywheel.setVoltage(-12.0);
            
        } else {
            flywheel.setVelocity(shootingParams.get().flywheelSpeed());

        }

        Logger.recordOutput("Superstructure/IsFirstActive", getIsFirstActive());
        Logger.recordOutput("Superstructure/IsHubActive", isHubActive());
        Logger.recordOutput("Superstructure/ShouldShootHub", shouldShootHub());
        Logger.recordOutput("Superstructure/FeedState", feedState);
        Logger.recordOutput("Superstructure/ShooterAtSetpoint", shooterAtSetpoint());
        Logger.recordOutput("Superstructure/ParamPreset", currPreset);
        Logger.recordOutput("Superstructure/PeriodCountdown", getPeriodCountDown());
        Logger.recordOutput("Superstructure/MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("Superstructure/PeriodName", getCurrentPeriod());

    }

    public void setIsManual(boolean isManual) {
        this.isManual = isManual;
        SmartDashboard.putBoolean("Superstructure/IsManual", isManual);
    }

    public boolean getIsManual() {
        return SmartDashboard.getBoolean("Superstructure/IsManual", isManual);
    }

    public void setIsFirstActive(boolean isFirstActive) {
        this.isFirstActive = isFirstActive;
        SmartDashboard.putBoolean("Superstructure/IsFirstActive", isFirstActive);
    }

    public boolean getIsFirstActive() {
        return SmartDashboard.getBoolean("Superstructure/IsFirstActive", isFirstActive);
    }

    public boolean isHubActive() {
        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous() || matchTime > 130 || matchTime < 30) {
            return true;
        }

        if (matchTime > 105 || (matchTime < 80 && matchTime > 55)) {
            return getIsFirstActive();
        } else {
            return !getIsFirstActive();
        }
    }

    // Arrow stacking :D
    public boolean shouldShootHub() {
        if (!isHubActive()) {
            return getPeriodCountDown() < hubPrefireTime.getAsDouble();
        }

        return true;
    }

    public double getPeriodCountDown() {
        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous()) {
            return matchTime;

        } else if (matchTime > 130) {
            return matchTime - 130;

        } else if (matchTime > 105) {
            return matchTime - 105;

        } else if (matchTime > 80) {
            return matchTime - 80;

        } else if (matchTime > 55) {
            return matchTime - 55;

        } else if (matchTime > 30) {
            return matchTime - 30;

        } else {
            return matchTime;
        }
    }

    public String getCurrentPeriod() {
        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous()) {
            return "Auto";

        } else if (matchTime > 130) {
            return "Transition";

        } else if (matchTime > 105) {
            return "Shift 1";

        } else if (matchTime > 80) {
            return "Shift 2";

        } else if (matchTime > 55) {
            return "Shift 3";

        } else if (matchTime > 30) {
            return "Shift 4";

        } else {
            return "Endgame";
        }
    }

    public boolean shooterAtSetpoint() {
        return Constants.isSim ||
                (flywheel.atSetpoint()
                        && (drive.getSystemState() == Drive.SystemState.HEADING_ALIGN ? drive.isAtHeading() : true));
    }

    public Command deployIntake() {
        return Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.INTAKE));
    }

    public Command retractIntake() {
        return Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.RETRACT));
    }

    public Command enableShooter() { // auto aim
        return Commands.runOnce(() -> {
            if (RobotState.getInstance().getCurrentFieldZone() == FieldZone.ALLIANCE) {
                shootingParams = ShotCalculator.getInstance()::getHubParameters;
                drive.setVelocityScalar(DriveSpeedScalar.CRAWL);
                if (isManual) {
                    drive.setHeadingAlignRotation(() -> shootingParams.get().robotHeading());

                } else {
                    drive.setAutoAlignPose(new Pose2d(2.9, FieldConstants.fieldWidth / 2.0, Rotation2d.kPi));

                }

                feedState = FeedState.ACTIVE;

            } else {
                shootingParams = ShotCalculator.getInstance()::getFerryParameters;
                drive.setWantedState(WantedState.TELEOP);

                feedState = FeedState.FERRY;

            }
        });
    }

    public Command disableShooter() { // stop shooting
        return Commands.runOnce(() -> {
            shootingParams = ParamPreset.STOW::getParams;
            drive.setVelocityScalar(DriveSpeedScalar.DEFAULT);
            drive.setWantedState(WantedState.TELEOP);
            currPreset = ParamPreset.STOW;
            feedState = FeedState.NO;

        });
    }

    public Command shootPreset(ParamPreset preset) { // rev up a few secs before active period; auto shoots once it
                                                     // begins
        return Commands.runOnce(() -> {
            currPreset = preset;
            shootingParams = currPreset::getParams;
            drive.setWantedState(WantedState.TELEOP);

            if (preset == ParamPreset.SHOWER || preset == ParamPreset.SHUB) {
                feedState = FeedState.ACTIVE;

                // drive.setHeadingAlignRotation(AllianceFlipUtil.apply(Rotation2d.kPi));

            } else if (preset == ParamPreset.SHERRY) {
                feedState = FeedState.FERRY;

                if (!isManual) {
                    // drive.setHeadingAlignRotation(preset.getParams().robotHeading());

                }
            }
        });
    }

    public Command turtle() { // go under trench/exhaust
        return Commands.runOnce(() -> {
            intake.setWantedState(Intake.WantedState.EXHAUST);
            drive.setWantedState(WantedState.TELEOP);
            currPreset = ParamPreset.TURTLE;
            shootingParams = currPreset::getParams;
            feedState = FeedState.NO;
        });
    }
}
