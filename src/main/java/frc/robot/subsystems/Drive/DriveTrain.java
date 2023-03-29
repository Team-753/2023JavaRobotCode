package frc.robot.subsystems.Drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Config;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
    
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule rearLeftModule;
    private SwerveModule rearRightModule;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(2, 2, 2);
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
    private double speedLimitingFactor = 1;
    public Command enableSpeedLimiterCommand;
    public Command disableSpeedLimiterCommand;
    public Command goXModeCommand;
    private AHRS navxAHRS;
    private double deltaTilt = 0;
    private double currentTilt = 0;
    private double tiltTimeStamp = 0;
    private Timer tiltTimer = new Timer();
    private boolean isRedAlliance = false;
    private Field2d kField2d = new Field2d();

    public DriveTrain() {
        navxAHRS = new AHRS();
        frontLeftModule = new SwerveModule(frc.robot.Config.DimensionalConstants.SwerveModuleConfigurations.get("frontLeftModule"));
        frontRightModule = new SwerveModule(frc.robot.Config.DimensionalConstants.SwerveModuleConfigurations.get("frontRightModule"));
        rearLeftModule = new SwerveModule(frc.robot.Config.DimensionalConstants.SwerveModuleConfigurations.get("rearLeftModule"));
        rearRightModule = new SwerveModule(frc.robot.Config.DimensionalConstants.SwerveModuleConfigurations.get("rearRightModule"));
        kinematics = new SwerveDriveKinematics(
            new Translation2d(frc.robot.Config.DimensionalConstants.trackWidth / 2, frc.robot.Config.DimensionalConstants.wheelBase / 2), // front left module
            new Translation2d(frc.robot.Config.DimensionalConstants.trackWidth / 2, -frc.robot.Config.DimensionalConstants.wheelBase / 2), // front right module
            new Translation2d(-frc.robot.Config.DimensionalConstants.trackWidth / 2, frc.robot.Config.DimensionalConstants.wheelBase / 2), // rear left module
            new Translation2d(-frc.robot.Config.DimensionalConstants.trackWidth / 2, -frc.robot.Config.DimensionalConstants.wheelBase / 2) // rear right module
            ); 
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, navxAHRS.getRotation2d(), getSwerveModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
        enableSpeedLimiterCommand = runOnce(() -> enableSpeedLimiter());
        disableSpeedLimiterCommand = runOnce(() -> disableSpeedLimiter());
        goXModeCommand = run(() -> goXMode());
        tiltTimer.start();
        SmartDashboard.putData("Field", kField2d);
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = {frontLeftModule.getSwerveModulePosition(), frontRightModule.getSwerveModulePosition(), rearLeftModule.getSwerveModulePosition(), rearRightModule.getSwerveModulePosition()};
        return positions;
    }

    @Override
    public void periodic() {
        super.periodic();
        poseEstimator.update(navxAHRS.getRotation2d(), getSwerveModulePositions());
        if (LimelightHelpers.getCurrentPipelineIndex(Config.DimensionalConstants.limelightName) == 0 && LimelightHelpers.getTV(Config.DimensionalConstants.limelightName)) {
            Pose3d poseToTag = LimelightHelpers.getCameraPose3d_TargetSpace(Config.DimensionalConstants.limelightName);
            double distance = poseToTag.toPose2d().getTranslation().getNorm();
            if (distance <= Config.DimensionalConstants.apriltagThresholdDistance) {
                if (isRedAlliance) {
                    poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiRed(Config.DimensionalConstants.limelightName), LimelightHelpers.getLatency_Capture(Config.DimensionalConstants.limelightName) + LimelightHelpers.getLatency_Pipeline(Config.DimensionalConstants.limelightName));
                }
                else {
                    poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(Config.DimensionalConstants.limelightName), LimelightHelpers.getLatency_Capture(Config.DimensionalConstants.limelightName) + LimelightHelpers.getLatency_Pipeline(Config.DimensionalConstants.limelightName));
                }
            }
        }
        Pose2d currentEstimatedPose = getEstimatedPose();
        double oldTilt = currentTilt;
        double oldTime = tiltTimeStamp;
        double pitch = navxAHRS.getPitch();
        double roll = navxAHRS.getRoll();
        double yaw = currentEstimatedPose.getRotation().getDegrees();
        currentTilt = Math.abs(roll * Math.cos(yaw)) + Math.abs(pitch * Math.sin(yaw));
        tiltTimeStamp = tiltTimer.get();
        deltaTilt = (currentTilt - oldTilt) / (tiltTimeStamp - oldTime);
        SmartDashboard.putNumber("X", currentEstimatedPose.getX());
        SmartDashboard.putNumber("Y", currentEstimatedPose.getY());
        SmartDashboard.putNumber("Rotation", currentEstimatedPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Tilt", currentTilt);
        SmartDashboard.putNumber("Delta Tilt", deltaTilt);
        if (isRedAlliance) {
            kField2d.setRobotPose(new Pose2d(Config.DimensionalConstants.fieldLength - currentEstimatedPose.getX(), Config.DimensionalConstants.fieldHeight - currentEstimatedPose.getY(), currentEstimatedPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))));
        }
        else {
            kField2d.setRobotPose(currentEstimatedPose);
        }
    }

    public double getDeltaTilt() {
        return deltaTilt;
    }

    public double getTilt() {
        return currentTilt;
    }



    public void setRedAlliance(boolean kIsRedAlliance) {
        if (kIsRedAlliance) {
            isRedAlliance = true;
        }
        else {
            isRedAlliance = false;
        }
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds actualChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(frontLeftModule.getSwerveModuleState(), frontRightModule.getSwerveModuleState(), rearLeftModule.getSwerveModuleState(), rearRightModule.getSwerveModuleState()), Rotation2d.fromRadians(-getEstimatedPose().getRotation().getRadians()));
    }

    private void enableSpeedLimiter() {
        speedLimitingFactor = Config.TeleoperatedConstants.toggleableSpeedModifier;
    }

    private void disableSpeedLimiter() {
        speedLimitingFactor = 1;
    }

    private void goXMode() {
        frontLeftModule.XMode();
        frontRightModule.XMode();
        rearLeftModule.XMode();
        rearRightModule.XMode();
    }

    public void coast() {
        frontLeftModule.Coast();
        frontRightModule.Coast();
        rearLeftModule.Coast();
        rearRightModule.Coast();
    }

    public void stationary() {
        frontLeftModule.Brake();
        frontRightModule.Brake();
        rearLeftModule.Brake();
        rearRightModule.Brake();
    }

    public void joystickDrive(CommandJoystick joystick) {
        double[] inputs = {joystick.getX(), joystick.getY(), joystick.getZ()};
        double[] outputs = normalizeInputs(inputs);
        double xVel = outputs[1] * Config.TeleoperatedConstants.maxVelocity * speedLimitingFactor;
        double yVel = outputs[0] * Config.TeleoperatedConstants.maxVelocity * speedLimitingFactor;
        double zVel = outputs[2] * Config.TeleoperatedConstants.maxAngularVelocity * Config.TeleoperatedConstants.turningSpeedModifier;
        if (xVel == 0 && yVel == 0 && zVel == 0) {
            stationary();
        }
        else {
            setSwerveStates(xVel, yVel, zVel);
        }
    }

    private double[] normalizeInputs(double[] inputs) {
        double[] outputs = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            double input = inputs[i];
            double threshold = Config.TeleoperatedConstants.joystickDeadzones[i];
            if (Math.abs(input) > threshold) {
                double adjsutedInput = (Math.abs(input) - threshold) / (1 - threshold);
                if (input < 0) {
                    adjsutedInput = -adjsutedInput;
                }
                outputs[i] = adjsutedInput;
            }
            else {
                outputs[i] = 0;
            }
        }
        return outputs;
    }

    private void setSwerveStates(double xVelocity, double yVelocity, double zVelocity) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, zVelocity, getEstimatedPose().getRotation()));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.TeleoperatedConstants.maxVelocity);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        rearLeftModule.setState(moduleStates[2]);
        rearRightModule.setState(moduleStates[3]);
    }

    public void resetPose(Pose2d poseToSet) {
        poseEstimator.resetPosition(navxAHRS.getRotation2d(), getSwerveModulePositions(), poseToSet);
    }
    public void dummyResetPose(Pose2d nada) {}

    public void PPDrive(ChassisSpeeds speeds) {
        if (Math.abs(speeds.vxMetersPerSecond) < Config.AutonomousConstants.lowestVelocity && Math.abs(speeds.vyMetersPerSecond) < Config.AutonomousConstants.lowestVelocity && Math.abs(speeds.omegaRadiansPerSecond) < Config.AutonomousConstants.lowestAngularVelocity) {
            stationary();
        }
        speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond; // not sure if this is totally necessary
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getEstimatedPose().getRotation()));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.AutonomousConstants.maxVelocity);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        rearLeftModule.setState(moduleStates[2]);
        rearRightModule.setState(moduleStates[3]);
    }
}