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
import frc.robot.Config;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
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
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);
    private NetworkTable limelightNetworkTable;
    private double speedLimitingFactor = 1;
    public Command enableSpeedLimiterCommand;
    public Command disableSpeedLimiterCommand;
    public Command goXModeCommand;
    private AHRS navxAHRS;
    private double deltaTilt = 0;
    private double currentTilt = 0;
    private double tiltTimeStamp = 0;
    private Timer tiltTimer;
    private boolean isRedAlliance = false;

    public DriveTrain(NetworkTable llTable) {
        limelightNetworkTable = llTable;
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
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, navxAHRS.getRotation2d(), getSwerveModulePositions(), null, stateStdDevs, visionMeasurementStdDevs);
        enableSpeedLimiterCommand = runOnce(() -> enableSpeedLimiter());
        disableSpeedLimiterCommand = runOnce(() -> disableSpeedLimiter());
        goXModeCommand = run(() -> goXMode());
        tiltTimer.start();
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = {frontLeftModule.getSwerveModulePosition(), frontRightModule.getSwerveModulePosition(), rearLeftModule.getSwerveModulePosition(), rearRightModule.getSwerveModulePosition()};
        return positions;
    }

    @Override
    public void periodic() {
        super.periodic();
        poseEstimator.update(navxAHRS.getRotation2d(), getSwerveModulePositions());
        if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 0 && LimelightHelpers.getTV("limelight")) {
            Pose3d poseToTag = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
            double distance = poseToTag.toPose2d().getTranslation().getNorm();
            if (distance <= Config.DimensionalConstants.apriltagThresholdDistance) {
                if (isRedAlliance) {
                    poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiRed("limelight"), LimelightHelpers.getLatency_Capture("limelight") + LimelightHelpers.getLatency_Pipeline("limelight"));
                }
                else {
                    poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue("limelight"), LimelightHelpers.getLatency_Capture("limelight") + LimelightHelpers.getLatency_Pipeline("limelight"));
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
        speedLimitingFactor = 0.5;
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

    private void coast() {
        frontLeftModule.Coast();
        frontRightModule.Coast();
        rearLeftModule.Coast();
        rearRightModule.Coast();
    }

    private void stationary() {
        frontLeftModule.Brake();
        frontRightModule.Brake();
        rearLeftModule.Brake();
        rearRightModule.Brake();
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

    public void PPDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getEstimatedPose().getRotation()));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.AutonomousConstants.maxVelocity);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        rearLeftModule.setState(moduleStates[2]);
        rearRightModule.setState(moduleStates[3]);
    }
}
