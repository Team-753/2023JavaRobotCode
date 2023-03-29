package frc.robot.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.configs.SwerveModuleConfig;

public class SwerveModule {

    public final String moduleName;
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final double absoluteOffset;
    private final AnalogEncoder analogEncoder;
    private final Rotation2d xAngle;

    private static double drivingGearRatio = 8.14;
    private static double turningGearRatio = 12.8;
    private static double wheelDiameter = 0.1016;


    public SwerveModule(SwerveModuleConfig swerveModuleConfig)
    {

        moduleName = swerveModuleConfig.name;
        driveMotor = new TalonFX(swerveModuleConfig.driveMotorID);
        turnMotor = new TalonFX(swerveModuleConfig.turnMotorID);
        analogEncoder = new AnalogEncoder(swerveModuleConfig.analogID);
        absoluteOffset = swerveModuleConfig.analogOffset;
        xAngle = swerveModuleConfig.xAngle;

        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        turnMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        turnMotorConfig.slot0.kP = 0.1;
        turnMotorConfig.slot0.kI = 0.0001;
        turnMotorConfig.slot0.kD = 0.0001;
        turnMotorConfig.slot0.kF = 0;
        turnMotorConfig.slot0.integralZone = 100;
        turnMotorConfig.slot0.allowableClosedloopError = 0;
        SupplyCurrentLimitConfiguration turnSupplyCurrentConfig = new SupplyCurrentLimitConfiguration();
        turnSupplyCurrentConfig.currentLimit = 30.0;
        turnMotorConfig.supplyCurrLimit = turnSupplyCurrentConfig;
        turnMotor.configAllSettings(turnMotorConfig, 50);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        driveMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        driveMotorConfig.slot0.kP = 0.0005;
        driveMotorConfig.slot0.kI = 0.0005;
        driveMotorConfig.slot0.kD = 0.0000;
        driveMotorConfig.slot0.kF = 0.045;
        driveMotorConfig.slot0.integralZone = 500;
        driveMotorConfig.slot0.allowableClosedloopError = 0;
        SupplyCurrentLimitConfiguration driveSupplyCurrentConfig = new SupplyCurrentLimitConfiguration();
        driveSupplyCurrentConfig.currentLimit = 37.5;
        driveMotorConfig.supplyCurrLimit = driveSupplyCurrentConfig;
        driveMotor.configAllSettings(driveMotorConfig, 50);

        driveMotor.setSelectedSensorPosition(0.0, 0, 50);
        turnMotor.setSelectedSensorPosition(getAbsolutePosition() * 2048 * turningGearRatio / 360, 0, 50);

    }

    private double getAbsolutePosition() {
        return (analogEncoder.getAbsolutePosition() * 360) - absoluteOffset;
    }

    private Rotation2d getIntegratedState() {
        return Rotation2d.fromDegrees(((turnMotor.getSelectedSensorPosition() % (2048 * turningGearRatio)) * 360) / (2048 * turningGearRatio));
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getIntegratedState());
        double velocity = desiredState.speedMetersPerSecond * drivingGearRatio * 2048 / (wheelDiameter * Math.PI * 10); // converting from m/s to ticks / 100ms
        if (velocity == 0.0) {
            driveMotor.set(ControlMode.PercentOutput, 0);
        }
        else {
            driveMotor.set(TalonFXControlMode.Velocity, -velocity);
        }
        double angleDegrees = desiredState.angle.getDegrees();
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }
        double turnPositionTicks = turnMotor.getSelectedSensorPosition();
        turnMotor.set(TalonFXControlMode.Position, (turnPositionTicks - (turnPositionTicks % (2048 * turningGearRatio))) + (angleDegrees * 2048 * turningGearRatio / 360));
    }

    public void Coast() {
        turnMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setNeutralMode(NeutralMode.Coast);
        Stop();
    }

    public void Brake() {
        turnMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        Stop();
    }

    public void XMode() {
        driveMotor.setNeutralMode(NeutralMode.Coast);
        setState(new SwerveModuleState(0.0, xAngle));
    }

    private void Stop() {
        turnMotor.set(ControlMode.PercentOutput, 0);
        driveMotor.set(ControlMode.PercentOutput, 0);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double distanceMeters = driveMotor.getSelectedSensorPosition() * wheelDiameter * Math.PI / (2048 * drivingGearRatio);
        Rotation2d angle = getIntegratedState();
        return new SwerveModulePosition(distanceMeters, angle);
    }

    public SwerveModuleState getSwerveModuleState() {
        double velocity = (driveMotor.getSelectedSensorVelocity() * 10 * wheelDiameter * Math.PI) / (2048 * drivingGearRatio);
        Rotation2d angle = getIntegratedState();
        return new SwerveModuleState(velocity, angle);
    }
}
