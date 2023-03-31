package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.Config;

public class ArmManualControlCommand extends CommandBase {
    private CommandXboxController xboxController;
    private Arm arm;

    public ArmManualControlCommand(Arm kArm, CommandXboxController kXboxController) {
        arm = kArm;
        xboxController = kXboxController;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double input = xboxController.getLeftY();
        if (Math.abs(input) > Config.ArmConstants.manualControlDeadzone) {
            double adjsutedInput = (Math.abs(input) - Config.ArmConstants.manualControlDeadzone) / (1 - Config.ArmConstants.manualControlDeadzone);
            if (input < 1) {
                adjsutedInput = -adjsutedInput;
            }
            input = adjsutedInput;
        }
        else {
            input = 0;
        }
        arm.manualControl(input, xboxController.getHID().getRightBumper());
    }
}
