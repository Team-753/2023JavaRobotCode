package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveTrain;

public class StayBalancedCommand extends CommandBase {
    private DriveTrain driveTrain;
    private static double oopsieThreshold = 4;
    private Timer driveTimer;
    private static double bangTime = 0.30;
    private static double bangVel = 0.375;

    public StayBalancedCommand(DriveTrain kDrive) {
        driveTrain = kDrive;
        driveTimer = new Timer();
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        if (Math.abs(driveTrain.getTilt()) > oopsieThreshold) {
            double modifier = 1.0;
            if (driveTrain.getTilt() < -4) {
                modifier = -1.0;
            driveTimer.start();
            if (driveTimer.hasElapsed(bangTime)) {
                driveTimer.stop();
                driveTimer.reset();
                driveTrain.goXMode();
            }
            else {
                driveTrain.setChassisSpeeds(bangVel * modifier, 0.0, 0.0, true);
                }
            }
        }
        else {
            driveTrain.goXMode();
        }
    }
}
