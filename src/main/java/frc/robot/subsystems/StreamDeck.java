package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class StreamDeck extends SubsystemBase {
    private int[] selectedGridSlot = {2, 5};
    private Arm arm;
    private GenericHID streamDeck;
    
    public StreamDeck(Arm kArm) {
        arm = kArm;
        streamDeck = new GenericHID(Config.TeleoperatedConstants.streamDeckPort);
        // SmartDashboard.putNumber("Target Grid", selectedGridSlot[0]);
        // SmartDashboard.putNumber("Target Slot", selectedGridSlot[1]);
    }

    @Override
    public void periodic() {
        // i hate that i have to do it like this but it is the best way
        for (int i = 1; i < 10; i++) {
            if (streamDeck.getRawButtonReleased(i)) {
                selectedGridSlot[1] = i;
                return;
            }
        }
        if (streamDeck.getRawButtonReleased(10)) {
            selectedGridSlot[0] = 1;
            return;
        }
        if (streamDeck.getRawButtonReleased(11)) {
            selectedGridSlot[0] = 2;
            return;
        }
        if (streamDeck.getRawButtonReleased(12)) {
            selectedGridSlot[0] = 3;
            return;
        }
        if (streamDeck.getRawButtonReleased(13)) {
            arm.setPosition("FullyRetracted");
            return;
        }
        if (streamDeck.getRawButtonReleased(14)) {
            arm.setPosition("Optimized");
            return;
        }
        if (streamDeck.getRawButtonReleased(15)) {
            arm.setPosition("Substation");
            return;
        }
        if (streamDeck.getRawButtonReleased(16)) {
            arm.setPosition("Floor");
            return;
        }
        if (streamDeck.getRawButtonReleased(17)) {
            arm.setPosition("BottomSlot");
            return;
        }
        if (streamDeck.getRawButtonReleased(18)) {
            arm.setPosition("MidCone");
            return;
        }
        if (streamDeck.getRawButtonReleased(19)) {
            arm.setPosition("MidCube");
            return;
        }
        if (streamDeck.getRawButtonReleased(20)) {
            arm.setPosition("HighCone");
            return;
        }
        if (streamDeck.getRawButtonReleased(21)) {
            arm.setPosition("HighCube");
            return;
        }
    }

    public int[] getSelectedGridSlot() {
        int grid = selectedGridSlot[0];
        int position = selectedGridSlot[1];
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            grid = 4 - grid;
            if (position > 6) { // low grid
                if (position == 9) {
                    position = 7;
                }
                else if (position == 7) {
                    position = 9;
                }
            }
            else if (position < 4) { // high grid
                if (position == 3) {
                    position = 1;
                }
                else if (position == 1) {
                    position = 3;
                }
            }
            else {
                if (position == 6) {
                    position = 4;
                }
                else if (position == 4) {
                    position = 6;
                }
            }
        }
        // int grid = (int) SmartDashboard.getNumber("Target Grid", 1);
        // int position = (int) SmartDashboard.getNumber("Target Slot", 5);
        int[] target = {grid - 1, position - 1};
        return target;
    }
}
