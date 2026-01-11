package frc.team4276.frc2026.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.lib.hid.ViXController;

public class Superstructure extends SubsystemBase {
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final ViXController controller;

  public enum WantedSuperState {
    STOW,
    SHOOT
  }

  public enum CurrentSuperState {
    STOWED
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOW;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOWED;

  public Superstructure(Drive drive, Vision vision, ViXController controller) {
    this.drive = drive;
    this.vision = vision;
    this.controller = controller;
  }

  @Override
  public void periodic() {

    currentSuperState = handleStateTransition();
    applyState();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
  }

  private CurrentSuperState handleStateTransition() {
    return switch (wantedSuperState) {
      case STOW:
        yield CurrentSuperState.STOWED;
      case SHOOT:
        yield CurrentSuperState.STOWED;
    };
  }

  private void applyState() {
    switch (currentSuperState) {
      case STOWED:
        drive.setWantedState(Drive.WantedState.TELEOP);

        break;
    }
  }

  public void setWantedSuperState(WantedSuperState state) {
    wantedSuperState = state;
  }

  public Command setStateCommand(WantedSuperState superState) {
    return Commands.runOnce(() -> setWantedSuperState(superState));
  }

  public Command configureButtonBinding(
      WantedSuperState condition1) {

    return setStateCommand(condition1);
  }
}
