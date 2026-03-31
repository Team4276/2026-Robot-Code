package frc.team4276.frc2026.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.lib.VirtualSubsystem;
import frc.team4276.lib.geometry.AllianceFlipUtil;

import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoSelector extends VirtualSubsystem {
    private final AutoFactory autoFactory;

    private final LoggedDashboardChooser<Supplier<Command>> routineChooser = new LoggedDashboardChooser<>(
            "Comp/Auto/RoutineChooser");
    private Supplier<Command> lastRoutine = () -> Commands.none();
    private String lastRoutineName = "";

    private Command autoCommand;
    private static boolean autoChanged = true;

    private final LoggedNetworkNumber delayInput = new LoggedNetworkNumber("Comp/Auto/Delay", 0.0);
    private double prevDelayInput = 0.0;

    public AutoSelector(AutoFactory autoFactory) {
        this.autoFactory = autoFactory;

        routineChooser.addDefaultOption("Do Nothing", () -> this.autoFactory.idle());
        routineChooser.addOption("Chizu Depot", () -> this.autoFactory.chizu(true));
        routineChooser.addOption("Chizu Kobe", () -> this.autoFactory.chizu(false));
        routineChooser.addOption("Yuzu Depot", () -> this.autoFactory.yuzu(true));
        routineChooser.addOption("Yuzu Kobe", () -> this.autoFactory.yuzu(false));
        routineChooser.addOption("Yuzu Mint", () -> this.autoFactory.mintYuzu());
        routineChooser.addOption("Vanilla", () -> this.autoFactory.vanilla("Vanilla"));
        routineChooser.addOption("Vanilleft", () -> this.autoFactory.vanilla("Vanilleft"));
        routineChooser.addOption("Vaniright", () -> this.autoFactory.vanilla("Vaniright"));
        routineChooser.addOption("VanillaMintSwirl", () -> this.autoFactory.vanillaMintSwirl("Vanilleft"));
        // routineChooser.addOption("VanillaWithSprinkles", () -> this.autoFactory.vanillaWithSprinkles("Vaniright"));
        // routineChooser.addOption("VanillaMintSwirlWithSprinkles",
        //         () -> this.autoFactory.vanillaMintSwirlWithSprinkles("Vanilleft"));
        // routineChooser.addOption("RockyRoadRight", () -> this.autoFactory.rockyRoad(false, false));
        // routineChooser.addOption("RockyRoadSwipeRight", () -> this.autoFactory.rockyRoad(false, true));
        // routineChooser.addOption("RockyRoadLeft", () -> this.autoFactory.rockyRoad(true, false));
        // routineChooser.addOption("RockyRoadSwipeLeft", () -> this.autoFactory.rockyRoad(true, true));
    }

    /** Returns the selected auto command with the inputted delay. */
    public Command getCommand() {
        if (autoCommand == null) {
            autoCommand = lastRoutine
                    .get()
                    .beforeStarting(Commands.waitSeconds(getDelayInput()))
                    .finallyDo(() -> this.autoFactory.autoEnd());
        }

        return autoCommand;
    }

    public double getDelayInput() {
        return delayInput.get();
    }

    private boolean wasRed = false;

    public void periodic() {
        // Skip updates when actively running in auto
        if (DriverStation.isAutonomousEnabled() && lastRoutine != null) {
            return;
        }

        autoChanged = false;

        SmartDashboard.putNumber("Comp/Auto/Delay Input Submitted ", getDelayInput());

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
            autoChanged = true;
        }

        SmartDashboard.putString("Comp/Auto/Routine Submitted ", lastRoutineName);

        if (AllianceFlipUtil.shouldFlip() != wasRed) {
            autoChanged = true;
        }

        if (getDelayInput() == prevDelayInput) {
            autoChanged = true;
        }

        wasRed = AllianceFlipUtil.shouldFlip();

        if (autoChanged) {
            autoCommand = null;
        }
    }

    public boolean autoChanged(){
        return autoChanged;
    }
}
