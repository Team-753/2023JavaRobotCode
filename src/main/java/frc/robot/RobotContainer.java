// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Mandible;
import frc.robot.commands.ArmConfirmPositionCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MandibleOuttakeCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.subsystems.Arm;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

public class RobotContainer {
  private DriveTrain driveTrain;
  private Mandible mandible;
  private Arm arm;
  private SwerveAutoBuilder swerveAutoBuilder;
  private SwerveAutoBuilder bareBonesAutoBuilder;
  private HashMap<String, Command> eventMap;
  private CommandJoystick joystick;
  private CommandXboxController xboxController;
  private String[] pathnames;
  public SendableChooser<String> autoChooser = new SendableChooser<>();
  private PathConstraints autoPathConstraints;

  public RobotContainer() {
    if (Config.AutonomousConstants.usePPServer) {
      PathPlannerServer.startServer(5811);
    }
    // initialize subsystems
    driveTrain = new DriveTrain();
    arm = new Arm();
    mandible = new Mandible();


    // initialize inputs
    joystick = new CommandJoystick(Config.TeleoperatedConstants.joystickPort);
    xboxController = new CommandXboxController(Config.TeleoperatedConstants.xboxControllerPort);


    // set default command
    driveTrain.setDefaultCommand(new DefaultDriveCommand(joystick, driveTrain));
    // mandible default command is set internally
    // arm doesn't have a default command, is triggerd on a given stick input

    autoPathConstraints = new PathConstraints(Config.AutonomousConstants.maxVelocity, Config.AutonomousConstants.maxAccel);
    // we need this for on the fly generation w/ no eventmap and no alliance color 
    bareBonesAutoBuilder = new SwerveAutoBuilder(driveTrain::getEstimatedPose, driveTrain::resetPose, new PIDConstants(Config.AutonomousConstants.translationKP, Config.AutonomousConstants.translationKI, Config.AutonomousConstants.translationKD), new PIDConstants(Config.AutonomousConstants.rotationKP, Config.AutonomousConstants.rotationKI, Config.AutonomousConstants.rotationKD), driveTrain::PPDrive, null, false, driveTrain);
    generateEventMap();
    swerveAutoBuilder = new SwerveAutoBuilder(driveTrain::getEstimatedPose, driveTrain::dummyResetPose, new PIDConstants(Config.AutonomousConstants.translationKP, Config.AutonomousConstants.translationKI, Config.AutonomousConstants.translationKD), new PIDConstants(Config.AutonomousConstants.rotationKP, Config.AutonomousConstants.rotationKI, Config.AutonomousConstants.rotationKD), driveTrain::PPDrive, eventMap, true, driveTrain);
    configureBindings();

    // setting our autos
    autoChooser.setDefaultOption("Place Cube", "Place Cube");
    autoChooser.addOption("Charge", "Charge");
    // Grabbing the auto path names as to automatically populate our dashboard
    // File f = new File(System.getProperty("user.dir") + "/src/main/deploy/pathplanner");
    // pathnames = f.list();
    // for (int i = 0; i < pathnames.length; i++) {
    //   pathnames[i] = pathnames[i].replace(".path", "");
    //   autoChooser.addOption(pathnames[i], pathnames[i]);
    // }
    autoChooser.addOption("Testing", "Testing");
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }

  private double getStickRemainder(double input) {
    double adjsutedInput = (Math.abs(input) - Config.ArmConstants.manualControlDeadzone) / (1 - Config.ArmConstants.manualControlDeadzone);
    if (input < 1) {
      adjsutedInput = -adjsutedInput;
    }
    return adjsutedInput;
  }

  private void configureBindings() {
    Trigger joystickFour = joystick.button(4);
    joystickFour.whileTrue(driveTrain.goXModeCommand);

    Trigger joystickTwelve = joystick.button(12);
    joystickTwelve.onTrue(driveTrain.enableSpeedLimiterCommand);
    joystickTwelve.onFalse(driveTrain.disableSpeedLimiterCommand);

    Trigger joystickEleven = joystick.button(11);
    joystickEleven.onTrue(Commands.runOnce(() -> driveTrain.resetPose(new Pose2d()), driveTrain));

    xboxController.x().onTrue(Commands.runOnce(() -> mandible.setOpen(false), mandible));
    xboxController.b().onTrue(Commands.runOnce(() -> mandible.setOpen(true), mandible));
    xboxController.a().whileTrue(mandible.toggleIntakeInCommand);
    xboxController.y().whileTrue(mandible.toggleIntakeOutCommand);
    xboxController.axisGreaterThan(0, Config.ArmConstants.manualControlDeadzone).whileTrue(Commands.run(()-> arm.manualControl(getStickRemainder(xboxController.getLeftY()), xboxController.getHID().getRightBumper()), arm));
    xboxController.axisLessThan(0, -Config.ArmConstants.manualControlDeadzone).whileTrue(Commands.run(()-> arm.manualControl(getStickRemainder(xboxController.getLeftY()), xboxController.getHID().getRightBumper()), arm));
  }

  public Command getAutonomousCommand() {
    SmartDashboard.putBoolean("Autonomous Finished", false);
    String autoName = autoChooser.getSelected();
    Command command;
    switch (autoName) {
      case "Charge":
        command = new SequentialCommandGroup();
        break;
      case "Place Cube":
        command = eventMap.get("Place Cube");
        break;
      default:
        command = swerveAutoBuilder.fullAuto(PathPlanner.loadPathGroup(autoName, autoPathConstraints));
    }
    return new SequentialCommandGroup(command, Commands.runOnce(() -> SmartDashboard.putBoolean("Autonomous Finished", true)));
  }

  public void generateEventMap() {
    eventMap = new HashMap<String, Command>();
    eventMap.put("Place Cube", new SequentialCommandGroup(new ArmConfirmPositionCommand(arm, "HighCube"), new MandibleOuttakeCommand(mandible), new SetArmPositionCommand(arm, "Optimized")));
    eventMap.put("Mandible In", mandible.toggleIntakeInCommand);
    eventMap.put("Mandible Out", mandible.toggleIntakeOutCommand);
    eventMap.put("Mandible Off", mandible.toggleIntakeOffCommand);
    eventMap.put("Open Mandible", Commands.runOnce(() -> mandible.setOpen(true), mandible));
    eventMap.put("Close Mandible", Commands.runOnce(() -> mandible.setOpen(false), mandible));
  }

  public void disabledPeriodic() {
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      driveTrain.setRedAlliance(true);
    }
    else {
      driveTrain.setRedAlliance(false);
    }
  }
}
