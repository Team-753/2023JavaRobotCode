package frc.robot.commands.AutonomousPickup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.LimelightHelper;
import frc.robot.subsystems.Mandible;
import frc.robot.subsystems.Drive.DriveTrain;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LockOnPieceCommand extends CommandBase {
    private DriveTrain driveTrain;
    private Mandible mandible;
    private TrapezoidProfile.State targetState = new TrapezoidProfile.State(0, 0); // in radians
    private ProfiledPIDController angleController;
    private NetworkTableInstance table;
    
    public LockOnPieceCommand(DriveTrain kDriveTrain, Mandible kMandible) {
        driveTrain = kDriveTrain;
        mandible = kMandible;
        angleController = new ProfiledPIDController(Config.DriveConstants.AutoPiecePickup.turnCommandP, Config.DriveConstants.AutoPiecePickup.turnCommandI, Config.DriveConstants.AutoPiecePickup.turnCommandD, Config.DriveConstants.AutoPiecePickup.turnControllerConstraints);
        angleController.setTolerance(Config.DriveConstants.AutoPiecePickup.turnCommandAngleTolerance, Config.DriveConstants.AutoPiecePickup.turnCommandVelocityTolerance);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(driveTrain, mandible);
        table = NetworkTableInstance.create();
    }

    @Override
    public void initialize() {
        LimelightHelper.setPipelineIndex(Config.DimensionalConstants.limelightName, 2); // index of two is our game piece pipeline
    }

    @Override
    public void execute() {
        System.out.println(LimelightHelper.getTV(Config.DimensionalConstants.limelightName));
        if (LimelightHelper.getTV(Config.DimensionalConstants.limelightName)) {
            // we see a valid target; lets aim at it
            //double ID = LimelightHelper.getNeuralClassID(Config.DimensionalConstants.limelightName);
            String ID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tclass").getString("Nada");
            System.out.println(ID);
            if (ID.strip().equals("cone")) {
                // it is a cone
                mandible.setOpen(false);
            }
            else {
                // it is a cube
                mandible.setOpen(true);
            }
            double response = angleController.calculate(Math.toRadians(LimelightHelper.getTX(Config.DimensionalConstants.limelightName)), targetState); // trying to aim to 0 degrees
            driveTrain.setChassisSpeeds(0.0, 0.0, response, true);
        }
        else {
            // well we can't really do much so let's do nothing
            driveTrain.stationary();
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stationary();
        LimelightHelper.setPipelineIndex(Config.DimensionalConstants.limelightName, 0);
    }

    @Override
    public boolean isFinished() {
        return angleController.atGoal();
    }

    
}
