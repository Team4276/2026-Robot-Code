package frc.team4276.frc2026.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team4276.frc2026.RobotState;
import frc.team4276.frc2026.FieldConstants.FieldZone;
import frc.team4276.frc2026.shooter.ShotCalculator;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.feeder.Feeder;
import frc.team4276.frc2026.subsystems.flywheel.Flywheel;
import frc.team4276.frc2026.subsystems.intake.Intake;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.lib.hid.ViXController;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Feeder feeder;
  private final Flywheel flywheel;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final ViXController controller;

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

  public Superstructure(
      Drive drive,
      Intake intake,
      Feeder feeder,
      Flywheel flywheel,
      Vision vision,
      ViXController controller) {
    this.drive = drive;
    this.intake = intake;
    this.feeder = feeder;
    this.flywheel = flywheel;
    this.vision = vision;
    this.controller = controller;

    activeRumble
        .onTrue(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 0.25, 3))
        .onFalse(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 1.0, 1));
  }

  @Override
  public void periodic() {
    if (shooterAtSetpoint()) {
      if (feedState == FeedState.ACTIVE && isHubActive()) {
        feeder.setSystemState(Feeder.SystemState.FEED);

      } else if (feedState == FeedState.FERRY) {
        feeder.setSystemState(Feeder.SystemState.FEED);
      } else {
        feeder.setSystemState(Feeder.SystemState.IDLE);

      }

    } else {
      feeder.setSystemState(Feeder.SystemState.IDLE);

    }

    if(feedState == FeedState.ACTIVE){
      drive.setHeadingAlignRotation(shootingParams.get().robotHeading());
    }

    flywheel.setVelocity(shootingParams.get().flywheelSpeed());

    Logger.recordOutput("Superstructure/IsFirstActive", isFirstActive);
    Logger.recordOutput("Superstructure/IsHubActive", isHubActive());
    Logger.recordOutput("Superstructure/FeedState", feedState);
    Logger.recordOutput("Superstructure/ShooterAtSetpoint", shooterAtSetpoint());
    Logger.recordOutput("Superstructure/ParamPreset", currPreset);

  }

  public void setIsFirstActive(boolean isFirstActive) {
    this.isFirstActive = isFirstActive;
  }

  public boolean isHubActive() {
    if (!DriverStation.isFMSAttached()) {
      return true;
    }

    double matchTime = DriverStation.getMatchTime();

    if (DriverStation.isAutonomous() || matchTime > 130 || matchTime < 30) {
      return true;
    }

    if (matchTime > 105 || (matchTime < 80 && matchTime > 55)) {
      return isFirstActive;
    } else {
      return !isFirstActive;
    }
  }

  public boolean shooterAtSetpoint() {
    return flywheel.atSetpoint();
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

        feedState = FeedState.ACTIVE;

      } else {
        shootingParams = ShotCalculator.getInstance()::getFerryParameters;

        feedState = FeedState.FERRY;

      }
    });
  }

  public Command disableShooter() { // stop feeding; keep inertia and target
    return Commands.runOnce(() -> {
      shootingParams = ParamPreset.STOW::getParams;
      currPreset = ParamPreset.STOW;
      feedState = FeedState.NO;

    });
  }

  public Command shootPreset(ParamPreset preset) { // rev up a few secs before active period; auto shoots once it begins
    return Commands.runOnce(() -> {
      currPreset = preset;
      shootingParams = currPreset::getParams;

      if (preset == ParamPreset.SHOWER || preset == ParamPreset.SHUB) {
        feedState = FeedState.ACTIVE;

        // drive.setHeadingAlignRotation(AllianceFlipUtil.apply(Rotation2d.kPi));

      } else if (preset == ParamPreset.SHERRY) {
        feedState = FeedState.FERRY;

        // drive.setHeadingAlignRotation(AllianceFlipUtil.apply(Rotation2d.kZero));

      }
    });
  }

  public Command turtle() { // go under trench
    return Commands.runOnce(() -> {
      intake.setWantedState(Intake.WantedState.INTAKE);
      currPreset = ParamPreset.TURTLE;
      shootingParams = currPreset::getParams;
      feedState = FeedState.NO;
    });
  }
}
