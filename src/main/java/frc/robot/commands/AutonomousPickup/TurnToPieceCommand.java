package frc.robot.commands.AutonomousPickup;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.Drive.DriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class TurnToPieceCommand extends CommandBase {
    private TrapezoidProfile.State targetState; // in radians
    private ProfiledPIDController angleController;
    private DriveTrain driveTrain;

    public TurnToPieceCommand(DriveTrain kDrivetrain) {
        driveTrain = kDrivetrain;
        angleController = new ProfiledPIDController(Config.DriveConstants.turnCommandP, Config.DriveConstants.turnCommandI, Config.DriveConstants.turnCommandD, Config.DriveConstants.turnControllerConstraints);
        angleController.setTolerance(Config.DriveConstants.turnCommandAngleTolerance, Config.DriveConstants.turnCommandVelocityTolerance); // +/- 0.5 degrees (yes value is converted to radians)
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        targetState = new TrapezoidProfile.State(getNearestPieceAngle().getRadians(), 0);
    }

    @Override
    public void execute() {
        double output = angleController.calculate(driveTrain.getEstimatedPose().getRotation().getRadians(), targetState);
        driveTrain.setChassisSpeeds(0.0, 0.0, output, true);
        //System.out.println(String.format("Output %f", output));
        //System.out.println(String.format("Target %f", Math.toDegrees(targetState.position)));
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stationary();
    }

    @Override
    public boolean isFinished() {
        return angleController.atGoal() || Config.DEBUGGING.bypassAutoChecks || Config.DEBUGGING.bypassAutoTurnTo;
    }

    private Rotation2d getNearestPieceAngle() {
        double[] gamePieceYValues = Config.DriveConstants.AutoPiecePickup.gamePieceYValues;
        double[] verticalStack = {(gamePieceYValues[0] + gamePieceYValues[1]) / 2, (gamePieceYValues[1] + gamePieceYValues[2]) / 2, (gamePieceYValues[2] + gamePieceYValues[3]) / 2};
        Pose2d currentPose2d = driveTrain.getEstimatedPose();
        double currentY = currentPose2d.getY();
        double targetGamePieceY;
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
          currentY = Config.DimensionalConstants.fieldHeight - currentY; // re-orienting it back to blue alliance for the math
        }
        if (currentY <= verticalStack[0]) {
          targetGamePieceY = gamePieceYValues[0]; // bottom game piece
        }
        else if (currentY > verticalStack[0] && currentY <= verticalStack[1]) {
          targetGamePieceY = gamePieceYValues[1]; // 2nd game piece from the bottom
        }
        else if (currentY > verticalStack[1] && currentY <= verticalStack[2]) {
          targetGamePieceY = gamePieceYValues[2]; // 3rd game piece from the bottom
        }
        else {
          targetGamePieceY = gamePieceYValues[3]; // 4th game piece from the bottom
        }
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
          targetGamePieceY = Config.DimensionalConstants.fieldHeight - targetGamePieceY; // re-orienting it back to blue alliance for the math
        }
        Rotation2d thingy = new Rotation2d(currentPose2d.getX() - Config.DriveConstants.AutoPiecePickup.gamePieceXValue, currentPose2d.getY() - targetGamePieceY); // the angle computed towards the game piece
        return thingy.rotateBy(Rotation2d.fromRadians(Math.PI));
      }
}
