package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class Arm extends SubsystemBase {
    private double targetValue = 0;
    private double maxHeightInches = 42.5;
    private double encoderTicksToDistanceConversionFactor = 1 / (2 * 4 * 2048);
    private boolean zeroed;
    private DigitalInput limitSwitch;
    private TalonFX armFalcon;

    public Arm() {
        limitSwitch = new DigitalInput(Config.ArmConstants.limitSwitchID);
        armFalcon = new TalonFX(Config.ArmConstants.falconID);
        TalonFXConfiguration armFalconConfig = new TalonFXConfiguration();
        armFalconConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        armFalconConfig.slot0.kP = 0.1;
        armFalconConfig.slot0.kI = 0.0001;
        armFalconConfig.slot0.kD = 0.0001;
        armFalconConfig.slot0.kF = 0;
        armFalconConfig.slot0.integralZone = 100;
        armFalconConfig.slot0.allowableClosedloopError = 256;
        SupplyCurrentLimitConfiguration armSupplyCurrentConfig = new SupplyCurrentLimitConfiguration();
        armSupplyCurrentConfig.currentLimit = 30.0;
        armFalconConfig.supplyCurrLimit = armSupplyCurrentConfig;
        armFalcon.configAllSettings(armFalconConfig, 50);
        armFalcon.setNeutralMode(NeutralMode.Coast);
        zeroed = false;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (!zeroed) {
            if (limitSwitch.get()) {
                armFalcon.set(TalonFXControlMode.PercentOutput, 0);
                armFalcon.setSelectedSensorPosition(0, 0, 50);
                zeroed = true;
            }
            else {
                armFalcon.set(TalonFXControlMode.PercentOutput, -0.25);
            }
        }
        else {
            armFalcon.set(TalonFXControlMode.Position, targetValue / encoderTicksToDistanceConversionFactor);
            SmartDashboard.putNumber("Arm Position", armFalcon.getSelectedSensorPosition() * encoderTicksToDistanceConversionFactor);
        }
    }

    public void setPosition(String positionToSet) {
        targetValue = Config.ArmConstants.armValues.get(positionToSet);
    }

    public Boolean atSetpoint() {
        if (Math.abs(targetValue - (armFalcon.getSelectedSensorPosition() * encoderTicksToDistanceConversionFactor)) < Config.ArmConstants.autoPlacementTolerance) {
            return true;
        }
        return false;
    }
}
