package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.playingwithfusion.TimeOfFlight;
//import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.TimeOfFlight.RangingMode;


public class Mandible extends SubsystemBase {
    public Boolean open = true;
    private DoubleSolenoid actuator;
    private Compressor compressor;
    private VictorSPX leftMotor;
    private VictorSPX rightMotor;
    public TimeOfFlight distanceSensor;
    public Command toggleIntakeInCommand;
    public Command toggleIntakeOffCommand;
    public Command toggleIntakeOutCommand;

    public Mandible() {
        actuator = new DoubleSolenoid(Config.DimensionalConstants.pcmID, PneumaticsModuleType.CTREPCM, Config.MandibleConstants.forwardChannel, Config.MandibleConstants.reverseChannel);
        compressor = new Compressor(Config.DimensionalConstants.pcmID, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        leftMotor = new VictorSPX(Config.MandibleConstants.leftMotorID);
        rightMotor = new VictorSPX(Config.MandibleConstants.rightMotorID);
        rightMotor.setInverted(true);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        setDefaultCommand(run(() -> passiveIntake()));
        toggleIntakeInCommand = run(() -> intakeWheels());
        toggleIntakeOffCommand = runOnce(() -> passiveIntake());
        toggleIntakeOutCommand = run(() -> outtakeWheels());
        if (Config.MandibleConstants.useDistanceSensor) {
            distanceSensor = new TimeOfFlight(Config.MandibleConstants.distanceSensorID);
            distanceSensor.setRangingMode(RangingMode.Short, 24);
        }
    }

    public void passiveIntake() {
        if (open) {
            leftMotor.set(VictorSPXControlMode.PercentOutput, -Config.MandibleConstants.idleSpeed);
            rightMotor.set(VictorSPXControlMode.PercentOutput, -Config.MandibleConstants.idleSpeed);
        }
        else {
            stopWheels();
        }

    }

    // @Override
    // public void periodic() {
    //     super.periodic();
    //     double v1 = leftMotor.getMotorOutputVoltage();
    //     double v2 = rightMotor.getMotorOutputVoltage();
    //     SmartDashboard.putNumber("Mandible Avg. Voltage", (v1 + v2) / 2.0);
    // }

    private void Open() {
        open = true;
        actuator.set(DoubleSolenoid.Value.kReverse);
    }

    private void Close() {
        open = false;
        actuator.set(DoubleSolenoid.Value.kForward);
    }

    public void intakeWheels() {
        leftMotor.set(VictorSPXControlMode.PercentOutput, -Config.MandibleConstants.intakeSpeed);
        rightMotor.set(VictorSPXControlMode.PercentOutput, -Config.MandibleConstants.intakeSpeed);
    }

    public void outtakeWheels() {
        leftMotor.set(VictorSPXControlMode.PercentOutput, Config.MandibleConstants.outtakeSpeed);
        rightMotor.set(VictorSPXControlMode.PercentOutput, Config.MandibleConstants.outtakeSpeed);
    }

    private void stopWheels() {
        leftMotor.set(VictorSPXControlMode.PercentOutput, 0);
        rightMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void setOpen(boolean shouldOpen) {
        if (shouldOpen) {
            Open();
        }
        else {
            Close();
        }
    }

    // public void gamePieceInPossession() {
    //     // check voltage to see if motors are stalled
    // }
}