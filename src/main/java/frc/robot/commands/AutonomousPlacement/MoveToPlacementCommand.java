package frc.robot.commands.AutonomousPlacement;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.StreamDeck;
import frc.robot.subsystems.Drive.DriveTrain;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj.DriverStation;

public class MoveToPlacementCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final PPHolonomicDriveController controller;
    private PathPlannerTrajectory trajectory;
    private DriveTrain driveTrain;
    private SendableChooser<Integer> secondPieceChooser;
    private StreamDeck streamDeck;
    private SwerveDriveKinematics kinematics;
    boolean isAuto;

    public MoveToPlacementCommand(DriveTrain kDriveTrain, SendableChooser<Integer> kSecondPieceChooser) { // this one is for autonomous
        driveTrain = kDriveTrain;
        secondPieceChooser = kSecondPieceChooser;
        controller = new PPHolonomicDriveController(pidControllerFromConstants(Config.AutonomousConstants.translationConstants), pidControllerFromConstants(Config.AutonomousConstants.translationConstants), pidControllerFromConstants(Config.AutonomousConstants.rotationConstants));
        isAuto = true;
        kinematics = driveTrain.kinematics;
    }

    public MoveToPlacementCommand(DriveTrain kDriveTrain, StreamDeck kStreamDeck) { // this one is for teleoperated auto placement
        driveTrain = kDriveTrain;
        streamDeck = kStreamDeck;
        controller = new PPHolonomicDriveController(pidControllerFromConstants(Config.AutonomousConstants.translationConstants), pidControllerFromConstants(Config.AutonomousConstants.translationConstants), pidControllerFromConstants(Config.AutonomousConstants.rotationConstants));
        isAuto = false;
        kinematics = driveTrain.kinematics;
    }

    private PathPlannerTrajectory getTrajectory() {
        int grid;
        int slot;
        Pose2d currentPose = driveTrain.getEstimatedPose();
        if (isAuto) {
            double firstGridYCutoff = 1.905;
            double secondGridYCutoff = 3.5814;
            double currentY = currentPose.getY();
            if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
                currentY = Config.DimensionalConstants.fieldHeight - currentY; // re-orienting it back to blue alliance for the math
              }
            if (currentY <= firstGridYCutoff) {
                grid = 2;
            }
            else if (currentY >= firstGridYCutoff && currentY <= secondGridYCutoff) {
                grid = 1;
            }
            else {
                grid = 0;
            }
            slot = secondPieceChooser.getSelected();
        }
        else {
            int[] data = streamDeck.getSelectedGridSlot();
            grid = data[0];
            slot = data[1];
        }
        double x = Config.DimensionalConstants.gridLayout[grid][slot][0];
        double y = Config.DimensionalConstants.gridLayout[grid][slot][1];
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            y = Config.DimensionalConstants.fieldHeight - y;
          }
        double offset;
        if (slot < 3) { // we are placing high
            if (slot == 1) { // high cube
                offset = Config.DriveConstants.PlacementOffsets.highCubeOffset;
            }
            else { // we are placing high cone
                offset = Config.DriveConstants.PlacementOffsets.highConeOffset;
            }
        }
        else if (slot > 5) { // we are placing low
            offset = Config.DriveConstants.PlacementOffsets.lowCubeOffset;
        }
        else {
            if (slot == 4) { // mid cube
                offset = Config.DriveConstants.PlacementOffsets.midCubeOffset;
            }
            else {
                offset = Config.DriveConstants.PlacementOffsets.midConeOffset;
            }
        }
        Pose2d targetPose2d = new Pose2d(new Translation2d(x + offset, y), Rotation2d.fromRadians(Math.PI));
        SmartDashboard.putString("Target Pose", String.format("X: %f, Y: %f, Z: %f", targetPose2d.getX(), targetPose2d.getY(), targetPose2d.getRotation().getDegrees()));
        Rotation2d heading = new Rotation2d(targetPose2d.getX() - currentPose.getX(), targetPose2d.getY() - currentPose.getY());
        //PathPoint.fromCurrentHolonomicState(currentPose, driveTrain.actualChassisSpeeds())
        return PathPlanner.generatePath(Config.AutonomousConstants.onTheFlyConstraints, new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()), new PathPoint(targetPose2d.getTranslation(), Rotation2d.fromRadians(Math.PI), targetPose2d.getRotation()));
    }

    @Override
    public void initialize() {
        trajectory = getTrajectory();
        timer.reset();
        timer.start();
  
      PathPlannerServer.sendActivePath(trajectory.getStates());
    }
  
    @Override
    public void execute() {
      double currentTime = this.timer.get();
      PathPlannerState desiredState = (PathPlannerState) trajectory.sample(currentTime);
  
      Pose2d currentPose = driveTrain.getEstimatedPose();
  
      PathPlannerServer.sendPathFollowingData(
          new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
          currentPose);
  
      ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
      driveTrain.PPSetStates(kinematics.toSwerveModuleStates(targetChassisSpeeds));
    }
  
    @Override
    public void end(boolean interrupted) {
      this.timer.stop();
  
      if (interrupted
          || Math.abs(trajectory.getEndState().velocityMetersPerSecond) < 0.1) {
            driveTrain.stationary();;
      }
    }
  
    @Override
    public boolean isFinished() {
      return this.timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    protected static PIDController pidControllerFromConstants(PIDConstants constants) {
        return new PIDController(constants.kP, constants.kI, constants.kD, constants.period);
      }
}
