package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveTrain;

public class DriveInDirectionCommand extends CommandBase {
    private DriveTrain driveTrain;
    private double xVelocity;
    private double yVelocity;
    private double zVelocity;
    private boolean fieldOrient = true;

    public DriveInDirectionCommand(DriveTrain kDriveTrain, double kxVel, double kyVel, double kzVel, boolean kFieldOrient) {
        driveTrain = kDriveTrain;
        xVelocity = kxVel;
        yVelocity = kyVel;
        zVelocity = kzVel;
        fieldOrient = kFieldOrient;
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

}
