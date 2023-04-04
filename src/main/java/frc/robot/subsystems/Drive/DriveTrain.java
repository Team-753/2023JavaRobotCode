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
import frc.robot.LimelightHelper;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Vector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.util.ConcurrentModificationException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
    
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule rearLeftModule;
    private SwerveModule rearRightModule;
    public SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Math.toRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.75, 0.75, Math.toRadians(30));
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
    private ShuffleboardTab debuggingTab;
    private double xVelocity = 0;
    private double yVelocity = 0;
    private double zVelocity = 0;
    private PhotonCamera photonCamera;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private double[] ppSpeeds = {0, 0, 0};

    public DriveTrain() {
        SmartDashboard.putBoolean("isRedAlliance", false);
        navxAHRS = new AHRS();
        frontLeftModule = new SwerveModule(Config.DimensionalConstants.SwerveModuleConfigurations.get("frontLeftModule"));
        frontRightModule = new SwerveModule(Config.DimensionalConstants.SwerveModuleConfigurations.get("frontRightModule"));
        rearLeftModule = new SwerveModule(Config.DimensionalConstants.SwerveModuleConfigurations.get("rearLeftModule"));
        rearRightModule = new SwerveModule(Config.DimensionalConstants.SwerveModuleConfigurations.get("rearRightModule"));
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Config.DimensionalConstants.trackWidth / 2, Config.DimensionalConstants.wheelBase / 2), // front left module
            new Translation2d(Config.DimensionalConstants.trackWidth / 2, -Config.DimensionalConstants.wheelBase / 2), // front right module
            new Translation2d(-Config.DimensionalConstants.trackWidth / 2, Config.DimensionalConstants.wheelBase / 2), // rear left module
            new Translation2d(-Config.DimensionalConstants.trackWidth / 2, -Config.DimensionalConstants.wheelBase / 2) // rear right module
            ); 
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, navxAHRS.getRotation2d(), getSwerveModulePositions(), new Pose2d(0, 0, Rotation2d.fromRadians(Math.PI)), stateStdDevs, visionMeasurementStdDevs);
        enableSpeedLimiterCommand = runOnce(() -> enableSpeedLimiter());
        disableSpeedLimiterCommand = runOnce(() -> disableSpeedLimiter());
        goXModeCommand = run(() -> goXMode());
        tiltTimer.start();
        SmartDashboard.putData("Field", kField2d);
        if (Config.AutonomousConstants.usePhotonCamera) {
            photonCamera = new PhotonCamera(Config.AutonomousConstants.photonCameraName);
            try {
                fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, Config.AutonomousConstants.cameraTransformation);
                photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            } catch (IOException e) {
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                photonPoseEstimator = null;
            }
        }
        if (Config.DEBUGGING.useDebugTab) {
            debuggingTab = Shuffleboard.getTab("DEBUGGING");
            if (Config.DEBUGGING.reportChassisSpeeds) {
                debuggingTab.addNumber("vX", this::getXVelocity).withPosition(2, 0);
                debuggingTab.addNumber("vY", this::getYVelocity).withPosition(3, 0);
                debuggingTab.addNumber("vO", this::getZVelocity).withPosition(4, 0);
            }
            if (Config.DEBUGGING.reportSwervePositions) {
                debuggingTab.addNumber("FL ABS", frontLeftModule::getRawAbsolutePosition).withPosition(0, 0);
                debuggingTab.addNumber("FR ABS", frontRightModule::getRawAbsolutePosition).withPosition(1, 0);
                debuggingTab.addNumber("RL ABS", rearLeftModule::getRawAbsolutePosition).withPosition(0, 1);
                debuggingTab.addNumber("RR ABS", rearRightModule::getRawAbsolutePosition).withPosition(1, 1);
                debuggingTab.addNumber("FL INT", frontLeftModule::getIntegratedPosition).withPosition(0, 2);
                debuggingTab.addNumber("FR INT", frontRightModule::getIntegratedPosition).withPosition(1, 2);
                debuggingTab.addNumber("RL INT", rearLeftModule::getIntegratedPosition).withPosition(0, 3);
                debuggingTab.addNumber("RR INT", rearRightModule::getIntegratedPosition).withPosition(1, 3);
            }
            // if (Config.DEBUGGING.ppSpeedDebug) {
            //     debuggingTab.addDoubleArray("Pathplanner Speeds", this::getFormattedPPSpeeds).withPosition(2, 1).withSize(2, 1);
            // }
          }
    }

    private double getXVelocity() {
        return xVelocity;
    }
    private double getYVelocity() {
        return yVelocity;
    }
    private double getZVelocity() {
        return zVelocity;
    }

    // private double[] getFormattedPPSpeeds() {
    //     return ppSpeeds;
    // }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = {frontLeftModule.getSwerveModulePosition(), frontRightModule.getSwerveModulePosition(), rearLeftModule.getSwerveModulePosition(), rearRightModule.getSwerveModulePosition()};
        return positions;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (Config.AutonomousConstants.useLLForPoseEstimation) {
            try {
                if (LimelightHelper.getCurrentPipelineIndex(Config.DimensionalConstants.limelightName) == 0.0 && LimelightHelper.getTV(Config.DimensionalConstants.limelightName)) {
                    Pose3d poseToTag = LimelightHelper.getCameraPose3d_TargetSpace(Config.DimensionalConstants.limelightName);
                    Translation2d translation = poseToTag.toPose2d().getTranslation();
                    double distance = Math.hypot(translation.getX(), translation.getY());
                    SmartDashboard.putNumber("Tag Distance", distance);
                    if (isRedAlliance) {
                        poseEstimator.addVisionMeasurement(LimelightHelper.getBotPose2d_wpiRed(Config.DimensionalConstants.limelightName), LimelightHelper.getLatency_Capture(Config.DimensionalConstants.limelightName) + LimelightHelper.getLatency_Pipeline(Config.DimensionalConstants.limelightName));
                    }
                    else {
                        poseEstimator.addVisionMeasurement(LimelightHelper.getBotPose2d_wpiBlue(Config.DimensionalConstants.limelightName), LimelightHelper.getLatency_Capture(Config.DimensionalConstants.limelightName) + LimelightHelper.getLatency_Pipeline(Config.DimensionalConstants.limelightName));
                        // poseEstimator.resetPosition(navxAHRS.getRotation2d(), getSwerveModulePositions(), LimelightHelper.getBotPose2d_wpiBlue(Config.DimensionalConstants.limelightName));
                    }
                    // if (distance <= Config.DimensionalConstants.apriltagThresholdDistance) {

                    // }
                }
            }
            catch (ConcurrentModificationException e) {
                DriverStation.reportError("LL Concurrent Modification Exception", e.getStackTrace());
            }
        }
        if (photonPoseEstimator != null) {
            Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
            if (result.isPresent()) {
                EstimatedRobotPose estimation = result.get();
                if (estimation.targetsUsed.size() > 1) { // using multi-tag pnp, it is probably trustworthy
                    poseEstimator.addVisionMeasurement(estimation.estimatedPose.toPose2d(), estimation.timestampSeconds);
                }
                else { // only one tag is visible, we should only trust it if we are under the threshold distance
                    PhotonTrackedTarget target = estimation.targetsUsed.get(0);
                    double camToTarget = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
                    if (target.getPoseAmbiguity() < 0.10 && camToTarget < Config.DimensionalConstants.apriltagThresholdDistance) {
                        poseEstimator.addVisionMeasurement(estimation.estimatedPose.toPose2d(), estimation.timestampSeconds);
                    }
                }
            }
        }
        
        Pose2d currentEstimatedPose = poseEstimator.update(navxAHRS.getRotation2d(), getSwerveModulePositions());
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
        if (Config.DEBUGGING.useDebugTab && Config.DEBUGGING.reportChassisSpeeds) {
            updateChassisSpeeds();
        }
        // if (Config.DEBUGGING.useDebugTab && Config.DEBUGGING.reportSwervePositions) {
            
        // }
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
            SmartDashboard.putBoolean("isRedAlliance", kIsRedAlliance);
            if (photonPoseEstimator != null) {
                fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                photonPoseEstimator.setFieldTags(fieldLayout);
            }
        }
        else {
            isRedAlliance = false;
            SmartDashboard.putBoolean("isRedAlliance", kIsRedAlliance);
            if (photonPoseEstimator != null) {
                fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                photonPoseEstimator.setFieldTags(fieldLayout);
            }
        }
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds actualChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(frontLeftModule.getSwerveModuleState(), frontRightModule.getSwerveModuleState(), rearLeftModule.getSwerveModuleState(), rearRightModule.getSwerveModuleState()), Rotation2d.fromRadians(-getEstimatedPose().getRotation().getRadians()));
    }
    private void updateChassisSpeeds() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(frontLeftModule.getSwerveModuleState(), frontRightModule.getSwerveModuleState(), rearLeftModule.getSwerveModuleState(), rearRightModule.getSwerveModuleState()), Rotation2d.fromRadians(-getEstimatedPose().getRotation().getRadians()));
        xVelocity = speeds.vxMetersPerSecond;
        yVelocity = speeds.vyMetersPerSecond;
        zVelocity = speeds.omegaRadiansPerSecond;
    }

    private void enableSpeedLimiter() {
        speedLimitingFactor = Config.TeleoperatedConstants.toggleableSpeedModifier;
    }

    private void disableSpeedLimiter() {
        speedLimitingFactor = 1;
    }

    public void goXMode() {
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
        double[] inputs = {-joystick.getX(), -joystick.getY(), -joystick.getZ()};
        double[] outputs = normalizeInputs(inputs);
        double xVel = outputs[1] * Config.TeleoperatedConstants.maxVelocity * speedLimitingFactor;
        double yVel = outputs[0] * Config.TeleoperatedConstants.maxVelocity * speedLimitingFactor;
        double zVel = outputs[2] * Config.TeleoperatedConstants.maxAngularVelocity * Config.TeleoperatedConstants.turningSpeedModifier;
        if (xVel == 0.0 && yVel == 0.0 && zVel == 0.0) {
            stationary();
        }
        else {
            setSwerveStates(xVel, yVel, zVel);
        }
    }

    public void joystickDriveXYOnly(CommandJoystick joystick, double rotationFeedback) {
        double[] inputs = {-joystick.getX(), -joystick.getY(), 0.0};
        double[] outputs = normalizeInputs(inputs);
        double xVel = outputs[1] * Config.TeleoperatedConstants.maxVelocity * speedLimitingFactor;
        double yVel = outputs[0] * Config.TeleoperatedConstants.maxVelocity * speedLimitingFactor;
        if (xVel == 0.0 && yVel == 0.0 && rotationFeedback == 0.0) {
            stationary();
        }
        else {
            setSwerveStates(xVel, yVel, rotationFeedback);
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

    public void setChassisSpeeds(double xVelocity, double yVelocity, double zVelocity, boolean fieldOrient) {
        if (fieldOrient) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, zVelocity, getEstimatedPose().getRotation()));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.TeleoperatedConstants.maxVelocity);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        rearLeftModule.setState(moduleStates[2]);
        rearRightModule.setState(moduleStates[3]);
        }
        else {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xVelocity, yVelocity, zVelocity));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.TeleoperatedConstants.maxVelocity);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        rearLeftModule.setState(moduleStates[2]);
        rearRightModule.setState(moduleStates[3]);
        }
    }

    public void resetPose(Pose2d poseToSet) {
        if (Config.AutonomousConstants.usePoseReset) {
            poseEstimator.resetPosition(navxAHRS.getRotation2d(), getSwerveModulePositions(), poseToSet);
        }
    }

    public void resetPose(Pose2d poseToSet, boolean override) {
        poseEstimator.resetPosition(navxAHRS.getRotation2d(), getSwerveModulePositions(), poseToSet);
    }

    // public void PPDrive(ChassisSpeeds speeds) {
    //     if (Math.abs(speeds.vxMetersPerSecond) < Config.AutonomousConstants.lowestVelocity && Math.abs(speeds.vyMetersPerSecond) < Config.AutonomousConstants.lowestVelocity && Math.abs(speeds.omegaRadiansPerSecond) < Config.AutonomousConstants.lowestAngularVelocity) {
    //         stationary();
    //     }
    //     // speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
    //     // speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
    //     //speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond; // not sure if this is totally necessary
    //     //SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getEstimatedPose().getRotation()));
    //     SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    //     SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.AutonomousConstants.maxVelocity);
    //     frontLeftModule.setState(moduleStates[0]);
    //     frontRightModule.setState(moduleStates[1]);
    //     rearLeftModule.setState(moduleStates[2]);
    //     rearRightModule.setState(moduleStates[3]);
    // }

    public void PPSetStates(SwerveModuleState[] moduleStates) {
        //ChassisSpeeds speeds = kinematics.toChassisSpeeds(moduleStates);
        // speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
        // speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
        //moduleStates = kinematics.toSwerveModuleStates(speeds);
        if (Config.DEBUGGING.ppSpeedDebug) {
            ChassisSpeeds speeds = kinematics.toChassisSpeeds(moduleStates);
            ppSpeeds[0] = speeds.vxMetersPerSecond;
            ppSpeeds[1] = speeds.vyMetersPerSecond;
            ppSpeeds[2] = speeds.omegaRadiansPerSecond;
            SmartDashboard.putNumber("ppVx", ppSpeeds[0]);
            SmartDashboard.putNumber("ppVy", ppSpeeds[1]);
            SmartDashboard.putNumber("ppVO", ppSpeeds[2]);
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.AutonomousConstants.maxVelocity);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        rearLeftModule.setState(moduleStates[2]);
        rearRightModule.setState(moduleStates[3]);
    }
}
