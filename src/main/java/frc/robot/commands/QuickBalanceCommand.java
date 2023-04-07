package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveTrain;

public class QuickBalanceCommand extends CommandBase {
    private DriveTrain driveTrain;
    private static double tiltThreshold = 10;

    public QuickBalanceCommand(DriveTrain kDrive) {
        driveTrain = kDrive;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.setChassisSpeeds(0.5, 0.0, 0.0, true);
    }

    @Override
    public boolean isFinished() {
        if (driveTrain.getTilt() < tiltThreshold) {
            return true;
        }
        return false;
    }
}
