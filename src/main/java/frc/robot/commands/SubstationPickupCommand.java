package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.Config;

public class SubstationPickupCommand extends CommandBase {
    private TrapezoidProfile.State targetState; // in radians
    private ProfiledPIDController angleController;
    private DriveTrain driveTrain;
    private CommandJoystick joystick;

    public SubstationPickupCommand(DriveTrain kDriveTrain, CommandJoystick kJoystick) {
        driveTrain = kDriveTrain;
        joystick = kJoystick;
        angleController = new ProfiledPIDController(Config.DriveConstants.turnCommandP, Config.DriveConstants.turnCommandI, Config.DriveConstants.turnCommandD, Config.DriveConstants.turnControllerConstraints);
        angleController.setTolerance(Config.DriveConstants.turnCommandAngleTolerance, Config.DriveConstants.turnCommandVelocityTolerance); // +/- 0.5 degrees (yes value is converted to radians)
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        targetState = new TrapezoidProfile.State(0.0, 0.0);
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.joystickDriveXYOnly(joystick, angleController.calculate(driveTrain.getEstimatedPose().getRotation().getRadians(), targetState));
    }
}
