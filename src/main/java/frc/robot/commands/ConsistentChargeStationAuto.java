package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive.DriveTrain;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Config;

public class ConsistentChargeStationAuto extends CommandBase {
    private DriveTrain driveTrain;
    private Arm arm;
    private boolean finished = false;
    private TrapezoidProfile.State targetState; // in radians
    private ProfiledPIDController angleController;
    private double targetVelocity;
    private String targetArmPosition;
    private Timer timer;

    public ConsistentChargeStationAuto(DriveTrain kDriveTrain, Arm kArm, boolean secondStage, boolean inverted) {
        driveTrain = kDriveTrain;
        arm = kArm;
        timer = new Timer();
        addRequirements(driveTrain, arm);
        angleController = new ProfiledPIDController(Config.DriveConstants.turnCommandP, Config.DriveConstants.turnCommandI, Config.DriveConstants.turnCommandD, Config.DriveConstants.turnControllerConstraints);
        angleController.setTolerance(Config.DriveConstants.turnCommandAngleTolerance, Config.DriveConstants.turnCommandVelocityTolerance); // +/- 0.5 degrees (yes value is converted to radians)
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        if (secondStage) {
            targetVelocity = Config.AutonomousConstants.chargeFinalSpeed;
            targetArmPosition = "Substation";
        }
        else {
            targetVelocity = Config.AutonomousConstants.chargeInitialSpeed;
            targetArmPosition = "Optimized";
        }
        double targetAngle = Math.PI;
        if (inverted) {
            targetAngle = 0; // facing forward
            targetVelocity = -targetVelocity;
        }
        targetState = new TrapezoidProfile.State(targetAngle, 0);

    }

    public ConsistentChargeStationAuto(DriveTrain kDriveTrain, Arm kArm, boolean secondStage) {
        driveTrain = kDriveTrain;
        arm = kArm;
        timer = new Timer();
        addRequirements(driveTrain, arm);
        angleController = new ProfiledPIDController(Config.DriveConstants.turnCommandP, Config.DriveConstants.turnCommandI, Config.DriveConstants.turnCommandD, Config.DriveConstants.turnControllerConstraints);
        angleController.setTolerance(Config.DriveConstants.turnCommandAngleTolerance, Config.DriveConstants.turnCommandVelocityTolerance); // +/- 0.5 degrees (yes value is converted to radians)
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        if (secondStage) {
            targetVelocity = Config.AutonomousConstants.chargeFinalSpeed;
            targetArmPosition = "Substation";
        }
        else {
            targetVelocity = Config.AutonomousConstants.chargeInitialSpeed;
            targetArmPosition = "Optimized";
        }
        targetState = new TrapezoidProfile.State(Math.PI, 0);

    }

    @Override
    public void initialize() {
        timer.restart();
    }
    @Override
    public void execute() {
        double deltaTilt = driveTrain.getDeltaTilt();
        if (deltaTilt < Config.AutonomousConstants.chargeDAngleThreshold && timer.hasElapsed(1)) {
            finished = true;
        }
        else {
            driveTrain.setChassisSpeeds(targetVelocity, 0.0, 0.0, true); // angleController.calculate(driveTrain.getEstimatedPose().getRotation().getRadians()
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.goXMode();
        timer.stop();
        //arm.setPosition(targetArmPosition);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
    
}