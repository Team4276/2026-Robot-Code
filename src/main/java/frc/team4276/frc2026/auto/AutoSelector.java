package frc.team4276.frc2026.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.lib.VirtualSubsystem;
import frc.team4276.lib.geometry.AllianceFlipUtil;

import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoSelector extends VirtualSubsystem {
    private final AutoFactory autoFactory;

    private final LoggedDashboardChooser<Supplier<Command>> routineChooser = new LoggedDashboardChooser<>(
            "Comp/Auto/RoutineChooser");
    private Supplier<Command> lastRoutine = () -> Commands.none();
    private String lastRoutineName = "";

    private boolean shouldRefresh = false;

    private boolean wasRed = false;

    private final LoggedNetworkBoolean isDepotSideInput = new LoggedNetworkBoolean("Comp/Auto/isDepotSide", false);
    private final LoggedNetworkBoolean isYumeInput = new LoggedNetworkBoolean("Comp/Auto/isYume", false);
    private final LoggedNetworkNumber delayInput = new LoggedNetworkNumber("Comp/Auto/Delay", 0.0);

    private boolean prevIsDepotSideInput = false;
    private boolean prevIsYumeInput = false;
    private double prevDelayInput = 0.0;

    public AutoSelector(AutoFactory autoFactory) {
        this.autoFactory = autoFactory;

        routineChooser.addDefaultOption("Do Nothing", () -> this.autoFactory.idle());
        routineChooser.addOption("Chizu", () -> this.autoFactory.chizu(true));
        routineChooser.addOption("Cheesu", () -> this.autoFactory.chizu(true));
        routineChooser.addOption("Yuzu", () -> this.autoFactory.yuzu(true));
        routineChooser.addOption("Vanilla", () -> this.autoFactory.vanilla("Vanilla"));
        routineChooser.addOption("Vanilleft", () -> this.autoFactory.vanilla("Vanilleft"));
        routineChooser.addOption("Vaniright", () -> this.autoFactory.vanilla("Vaniright"));
        routineChooser.addOption("VanillaMintSwirl", () -> this.autoFactory.vanillaMintSwirl("Vanilleft"));
    }

    /** Returns the selected auto command with the inputted delay. */
    public Command getCommand() {
        return lastRoutine
                .get()
                .beforeStarting(Commands.waitSeconds(delayInput.getAsDouble()))
                .finallyDo(() -> this.autoFactory.autoEnd());
    }

    public void periodic() {
        // Skip updates when actively running in auto
        if (DriverStation.isAutonomousEnabled()) {
            return;
        }

        SmartDashboard.putNumber("Comp/Auto/Delay Input Submitted ", delayInput.getAsDouble());

        // Update the list of questions
        var routineName = routineChooser.getSendableChooser().getSelected();

        // Update the routine and responses
        if (lastRoutineName != routineName) {
            var selectedRoutine = routineChooser.get();
            if (selectedRoutine == null) {
                return;
            }

            lastRoutine = selectedRoutine;
            lastRoutineName = routineName;
            shouldRefresh = true;
        }

        SmartDashboard.putString("Comp/Auto/Routine Submitted ", lastRoutineName);

        if (AllianceFlipUtil.shouldFlip() != wasRed) {
            shouldRefresh = true;

            wasRed = AllianceFlipUtil.shouldFlip();
        }

        if (delayInput.getAsDouble() == prevDelayInput) {
            shouldRefresh = true;

            prevDelayInput = delayInput.getAsDouble();
        }

        if (isDepotSideInput.getAsBoolean() == prevIsDepotSideInput) {
            shouldRefresh = true;

            prevIsDepotSideInput = isDepotSideInput.getAsBoolean();
        }

        if (isYumeInput.getAsBoolean() == prevIsYumeInput) {
            shouldRefresh = true;

            prevIsYumeInput = isYumeInput.getAsBoolean();
        }
    }

    public boolean shouldRefresh() {
        if (shouldRefresh) {
            shouldRefresh = false;
            return true;

        }

        return false;
    }
}
