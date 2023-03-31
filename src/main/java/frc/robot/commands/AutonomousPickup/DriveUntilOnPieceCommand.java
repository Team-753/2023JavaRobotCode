package frc.robot.commands.AutonomousPickup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.Drive.DriveTrain;

public class DriveUntilOnPieceCommand extends CommandBase {
    private DriveTrain driveTrain;

    public DriveUntilOnPieceCommand(DriveTrain kDriveTrain) {
        driveTrain = kDriveTrain;
        addRequirements(driveTrain);
    }


    @Override
    public void execute() {
        driveTrain.setChassisSpeeds(Config.DriveConstants.AutoPiecePickup.piecePickupVelocity, 0.0, 0.0, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stationary();
    }

    @Override
    public boolean isFinished() {
        return driveTrain.getEstimatedPose().getX() >= Config.DriveConstants.AutoPiecePickup.gamePieceXValue || Config.DEBUGGING.bypassAutoChecks; // have we passed over the game piece line yet
    }

}
