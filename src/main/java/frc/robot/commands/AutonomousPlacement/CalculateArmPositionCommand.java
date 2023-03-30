package frc.robot.commands.AutonomousPlacement;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.StreamDeck;

public class CalculateArmPositionCommand extends CommandBase {

    Arm arm;
    StreamDeck streamDeck;
    SendableChooser<Integer> piecePlacementChooser;
    boolean stageTwo;
    boolean isAutonomous;
    boolean finished = false;

    public CalculateArmPositionCommand(Arm kArm, SendableChooser<Integer> kPiecePlacementChooser, boolean kStageTwo) { // for autonomous
        arm = kArm;
        piecePlacementChooser = kPiecePlacementChooser;
        stageTwo = kStageTwo;
        isAutonomous = true;
        addRequirements(arm);
    }

    public CalculateArmPositionCommand(Arm kArm, StreamDeck kStreamDeck, boolean kStageTwo) { // for teleop
        arm = kArm;
        streamDeck = kStreamDeck;
        stageTwo = kStageTwo;
        isAutonomous = false;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPosition(getArmPosition());
        if (!stageTwo) { // we are just setting the arm position and leaving it
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint() || finished;
    }

    private String getArmPosition() {
        int slot;
        if (isAutonomous) {
            slot = piecePlacementChooser.getSelected();
        }
        else {
            slot = streamDeck.getSelectedGridSlot()[1];
        }
        if (slot < 3) { // we are placing high
            if (slot == 1) { // high cube
                return "HighCube";
            }
            else if (stageTwo) { // high cone placement
                return "HighConePlacement";
            }
            return "HighConePrep"; // only option left
        }
        else if (slot > 5) { // we are placing low
            return "BottomPlacement";
        }
        if (slot == 4) { // mid cube
            return "MidCube";
        }
        else if (stageTwo) { // placing mid cone
            return "MidConePlacement";
        }
        return "MidConePrep";
    }
}
