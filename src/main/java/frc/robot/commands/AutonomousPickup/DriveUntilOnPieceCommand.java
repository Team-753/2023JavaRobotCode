package frc.robot.commands.AutonomousPickup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.Mandible;
import frc.robot.subsystems.Drive.DriveTrain;

public class DriveUntilOnPieceCommand extends CommandBase {
    private DriveTrain driveTrain;
    private Mandible mandible;

    public DriveUntilOnPieceCommand(DriveTrain kDriveTrain, Mandible kMandible) {
        driveTrain = kDriveTrain;
        mandible = kMandible;
        addRequirements(driveTrain, mandible);
    }

    @Override
    public void initialize() {
        mandible.intakeWheels();
    }

    @Override
    public void execute() {
        driveTrain.setChassisSpeeds(Config.DriveConstants.AutoPiecePickup.piecePickupVelocity, 0.0, 0.0, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stationary();
        mandible.passiveIntake();
    }

    @Override
    public boolean isFinished() {
        return driveTrain.getEstimatedPose().getX() >= Config.DriveConstants.AutoPiecePickup.gamePieceXValue || Config.DEBUGGING.bypassAutoChecks || (mandible.distanceSensor.isRangeValid() && mandible.distanceSensor.getRange() < Config.MandibleConstants.distanceRangeThreshold); // have we passed over the game piece line yet
    }

}
