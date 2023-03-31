package frc.robot.commands.AutonomousPlacement;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Drive.DriveTrain;

public class DriverConfirmCommand extends CommandBase {
    private GenericHID joystick;
    public DriverConfirmCommand(CommandJoystick kJoystick, DriveTrain kDriveTrain) {
        joystick = kJoystick.getHID();
        addRequirements(kDriveTrain); // so the robot doesn't yk; move.
    }

    @Override
    public boolean isFinished() {
        return joystick.getRawButtonReleased(1);
    }
}
