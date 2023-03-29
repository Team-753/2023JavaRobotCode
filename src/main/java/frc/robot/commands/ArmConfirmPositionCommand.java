package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmConfirmPositionCommand extends CommandBase {
    
    private Arm arm;
    private String position;

    public ArmConfirmPositionCommand(Arm kArm, String kPosition) {
        arm = kArm;
        position = kPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }
}
