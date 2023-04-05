package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Arm extends SubsystemBase {
    private double targetValue = 0.0;
    private double maxHeightInches = 42.5;
    private double falconEncoderTicksToDistanceConversionFactor = 1.0 / (2.0 * 4.0 * 2048.0);
    private double neoEncoderTicksToDistanceConversionFactor = 1.0 / (2.0 * 4.0);
    private boolean zeroed;
    private DigitalInput limitSwitch;
    private TalonFX armFalcon;
    private CANSparkMax armNEO;
    private RelativeEncoder neoEncoder;
    //private SparkMaxPIDController neoPID;
    private ProfiledPIDController pidController;

    public Arm() {
        limitSwitch = new DigitalInput(Config.ArmConstants.limitSwitchID);
        zeroed = false;
        if (Config.ArmConstants.useNEO) {
            armNEO = new CANSparkMax(Config.ArmConstants.armID, MotorType.kBrushless);
            armNEO.setIdleMode(IdleMode.kCoast);
            armNEO.setSmartCurrentLimit(80);
            armNEO.enableVoltageCompensation(12.0);
            neoEncoder = armNEO.getEncoder();
            pidController = new ProfiledPIDController(0.75, 0.0, 0.0, new TrapezoidProfile.Constraints(12, 12));
            // neoPID = armNEO.getPIDController();
            // neoPID.setFeedbackDevice(neoEncoder);
            // neoPID.setP(2.0);
            // neoPID.setI(0.5);
            // neoPID.setD(0.0);
            // neoPID.setFF(0.0);
            // neoPID.setIZone(400.0);
            // neoPID.setReference(targetValue, ControlType.kPosition);
        }
        else {
            armFalcon = new TalonFX(Config.ArmConstants.armID);
            TalonFXConfiguration armFalconConfig = new TalonFXConfiguration();
            armFalconConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            armFalconConfig.slot0.kP = 0.1;
            armFalconConfig.slot0.kI = 0.0001;
            armFalconConfig.slot0.kD = 0.0001;
            armFalconConfig.slot0.kF = 0.0;
            armFalconConfig.slot0.integralZone = 100.0;
            armFalconConfig.slot0.allowableClosedloopError = 256.0;
            SupplyCurrentLimitConfiguration armSupplyCurrentConfig = new SupplyCurrentLimitConfiguration();
            armSupplyCurrentConfig.currentLimit = 40.0;
            armFalconConfig.supplyCurrLimit = armSupplyCurrentConfig;
            armFalcon.configAllSettings(armFalconConfig, 50);
            armFalcon.setNeutralMode(NeutralMode.Coast);
        }

    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
        if (!zeroed) {
            if (limitSwitch.get()) {
                if (Config.ArmConstants.useNEO) {
                    armNEO.set(0.0);
                    neoEncoder.setPosition(0.0);
                }
                else {
                    armFalcon.set(TalonFXControlMode.PercentOutput, 0);
                    armFalcon.setSelectedSensorPosition(0, 0, 50);
                }
                zeroed = true;
            }
            else {
                if (Config.ArmConstants.useNEO) {
                    armNEO.set(-0.45);
                }
                else {
                    armFalcon.set(TalonFXControlMode.PercentOutput, -0.25);
                }
            }
        }
        else {
            if (Config.ArmConstants.useNEO) {
                double output = pidController.calculate(neoEncoder.getPosition() * neoEncoderTicksToDistanceConversionFactor, new TrapezoidProfile.State(targetValue, 0.0));
                armNEO.set(output);
                //SmartDashboard.putNumber("Arm Output", output);
                SmartDashboard.putNumber("Arm Position", neoEncoder.getPosition() * neoEncoderTicksToDistanceConversionFactor);
            }
            else {
                armFalcon.set(TalonFXControlMode.Position, targetValue / falconEncoderTicksToDistanceConversionFactor);
                SmartDashboard.putNumber("Arm Position", armFalcon.getSelectedSensorPosition() * falconEncoderTicksToDistanceConversionFactor);
            }
        }
    }

    public void setPosition(String positionToSet) {
        targetValue = Config.ArmConstants.armValues.get(positionToSet);
        SmartDashboard.putString("Arm String Position", positionToSet);
    }

    public Boolean atSetpoint() {
        if (Config.ArmConstants.useNEO) {
            if (Math.abs(targetValue - (neoEncoder.getPosition() * neoEncoderTicksToDistanceConversionFactor)) < Config.ArmConstants.autoPlacementTolerance) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            if (Math.abs(targetValue - (armFalcon.getSelectedSensorPosition() * falconEncoderTicksToDistanceConversionFactor)) < Config.ArmConstants.autoPlacementTolerance) {
                return true;
            }
            else {
                return false;
            }
        }
    }

    public void manualControl(double scalar, boolean fast) {
        if (fast) {
            double newValue = targetValue + scalar * Config.ArmConstants.armIncrement;
            if (newValue >= 0.0 && newValue <= maxHeightInches) {
                targetValue = newValue;
            }
            }
        else {
            double newValue = targetValue + scalar * Config.ArmConstants.armIncrement * Config.ArmConstants.armSlowdownFactor;
            if (newValue >= 0.0 && newValue <= maxHeightInches) {
                targetValue = newValue;
            }
        }
    }

    public void startArmMovement() {
        // only run if we KNOW the arm is zeroed
        zeroed = true;
        if (Config.ArmConstants.useNEO) {
            armNEO.set(1);
        }
        else {
            armFalcon.set(ControlMode.PercentOutput, 1);
        }
    }
}
