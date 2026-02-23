package frc.team4276.frc2026.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.feeder.Feeder;
import frc.team4276.frc2026.subsystems.flywheel.Flywheel;
import frc.team4276.frc2026.subsystems.hood.Hood;
import frc.team4276.frc2026.subsystems.intake.Intake;
import frc.team4276.frc2026.subsystems.spindexer.Spindexer;
import frc.team4276.frc2026.subsystems.turret.Turret;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.lib.hid.ViXController;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final ViXController controller;

  private Supplier<ShootingParameters> shootingParams = ParamPreset.STOW::getParams;

  public Superstructure(
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Feeder feeder,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Vision vision,
      ViXController controller) {
    this.drive = drive;
    this.intake = intake;
    this.spindexer = spindexer;
    this.feeder = feeder;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.vision = vision;
    this.controller = controller;
  }

  @Override
  public void periodic() {

  }

  private boolean isFirstActive = false;

  public void setIsFirstActive(boolean isFirstActive) {
    this.isFirstActive = isFirstActive;
  }

  public boolean isHubActive() {
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

  public Command deployIntake() {
    return Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.INTAKE));
  }

  public Command retractIntake() {
    return Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.RETRACT));
  }

  public Command enableShooter() { // auto aim
    return Commands.none();
  }

  public Command disableShooter() { // stop feeding; keep inertia and target
    return Commands.none();
  }

  public Command shootPreset(ParamPreset preset) { // rev up a few secs before active period; auto shoots once it begins
    return Commands.runOnce(() -> shootingParams = preset::getParams);
  }

  public Command turtle() { // go under trench
    return Commands.runOnce(() -> {
      intake.setWantedState(Intake.WantedState.INTAKE);
    });
  }
}
