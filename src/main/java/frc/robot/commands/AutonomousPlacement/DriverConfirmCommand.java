package frc.robot.commands.AutonomousPlacement;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Mandible;
import frc.robot.subsystems.Drive.DriveTrain;

public class DriverConfirmCommand extends CommandBase {
    private GenericHID joystick;
    private Mandible mandible;
    public DriverConfirmCommand(CommandJoystick kJoystick, DriveTrain kDriveTrain, Mandible kMandible) {
        joystick = kJoystick.getHID();
        mandible = kMandible;
        addRequirements(kDriveTrain); // so the robot doesn't yk; move.
    }

    @Override
    public boolean isFinished() {
        return joystick.getRawButtonReleased(1) || mandible.open;
    }
}
