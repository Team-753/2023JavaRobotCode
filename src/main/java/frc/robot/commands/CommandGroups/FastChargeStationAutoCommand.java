package frc.robot.commands.CommandGroups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveInDirectionCommand;
import frc.robot.commands.QuickBalanceCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.StayBalancedCommand;
import frc.robot.commands.TurnToCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive.DriveTrain;

public class FastChargeStationAutoCommand extends SequentialCommandGroup {

    public FastChargeStationAutoCommand(DriveTrain kDrive, Arm kArm) {
        addCommands(new SetArmPositionCommand(kArm, "FullyRetracted"),
        new TurnToCommand(kDrive, Rotation2d.fromRadians(0)),
        new DriveInDirectionCommand(kDrive, 3.25, 0.0, 0.0, true, 1.5),
        new QuickBalanceCommand(kDrive),
        new DriveInDirectionCommand(kDrive, -0.5, 0, 0, true, 0.375),
        new StayBalancedCommand(kDrive)
        );
    }
}
