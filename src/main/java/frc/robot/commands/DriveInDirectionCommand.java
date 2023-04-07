package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveTrain;

public class DriveInDirectionCommand extends CommandBase {
    private DriveTrain driveTrain;
    private double xVelocity;
    private double yVelocity;
    private double zVelocity;
    private boolean fieldOrient = true;
    private double timeToDrive = 0.0;
    private Timer driveTimer;

    public DriveInDirectionCommand(DriveTrain kDriveTrain, double kxVel, double kyVel, double kzVel, boolean kFieldOrient) {
        driveTrain = kDriveTrain;
        xVelocity = kxVel;
        yVelocity = kyVel;
        zVelocity = kzVel;
        fieldOrient = kFieldOrient;
        addRequirements(driveTrain);
    }

    public DriveInDirectionCommand(DriveTrain kDriveTrain, double kxVel, double kyVel, double kzVel, boolean kFieldOrient, double kTimeToDrive) {
        driveTrain = kDriveTrain;
        xVelocity = kxVel;
        yVelocity = kyVel;
        zVelocity = kzVel;
        fieldOrient = kFieldOrient;
        timeToDrive = kTimeToDrive;
        addRequirements(driveTrain);
    }

    public DriveInDirectionCommand(DriveTrain kDriveTrain, double kxVel, double kyVel, double kzVel) {
        driveTrain = kDriveTrain;
        xVelocity = kxVel;
        yVelocity = kyVel;
        zVelocity = kzVel;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.setChassisSpeeds(xVelocity, yVelocity, zVelocity, fieldOrient);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stationary();
    }

    @Override
    public boolean isFinished() {
        if (timeToDrive != 0.0) {
            return driveTimer.hasElapsed(timeToDrive);
        }
        return false;
    }

}
