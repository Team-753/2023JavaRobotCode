package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.Drive.SwerveModuleConfig;

public class Config {
    public static class DimensionalConstants {
        public static double wheelBase = 0.5969;
        public static double trackWidth = 0.4954524;
        public static HashMap<String, SwerveModuleConfig> SwerveModuleConfigurations = new HashMap<String, SwerveModuleConfig>();
        static {
            SwerveModuleConfigurations.put("frontLeftModule", new SwerveModuleConfig(1, 2, 0, 77.05, -135, "frontLeftModule"));
            SwerveModuleConfigurations.put("frontRightModule", new SwerveModuleConfig(3, 4, 1, 309.7, 135, "frontRightModule"));
            SwerveModuleConfigurations.put("rearRightModule", new SwerveModuleConfig(5, 6, 2, 227, 45, "rearLeftModule"));
            SwerveModuleConfigurations.put("rearLeftModule", new SwerveModuleConfig(7, 8, 3, 24.1, -45, "rearRightModule"));
        }
        public static String limelightName = "limelight";
        public static int pcmID = 61;
        public static double apriltagThresholdDistance = 2;
        public static double fieldHeight = 8.0137;
        public static double fieldLength = 16.54175;
        public static double[][][] gridLayout = {
            {
                {0.4191, 4.987417}, 
                {0.4191, 4.424426}, 
                {0.4191, 3.861435}, 
                {0.8509, 4.987417}, 
                {0.8509, 4.424426}, 
                {0.8509, 3.861435}, 
                {1.27635, 4.987417}, 
                {1.27635, 4.424426}, 
                {1.27635, 3.861435}
            }, 
            {
                {0.4191, 3.311017}, 
                {0.4191, 2.748026}, 
                {0.4191, 2.185035}, 
                {0.8509, 3.311017}, 
                {0.8509, 2.748026}, 
                {0.8509, 2.185035}, 
                {1.27635, 3.311017}, 
                {1.27635, 2.748026}, 
                {1.27635, 2.185035}
                }, 
            {
                {0.4191, 1.634617}, 
                {0.4191, 1.071626}, 
                {0.4191, 0.508635}, 
                {0.8509, 1.634617}, 
                {0.8509, 1.071626}, 
                {0.8509, 0.508635}, 
                {1.27635, 1.634617}, 
                {1.27635, 1.071626}, 
                {1.27635, 0.508635}
            }
        };
    }
    public static class TeleoperatedConstants {
        public static double maxVelocity = 4.00;
        public static double maxAngularVelocity = maxVelocity / Math.hypot(DimensionalConstants.trackWidth / 2, DimensionalConstants.wheelBase / 2);
        public static double turningSpeedModifier = 0.60;
        public static double toggleableSpeedModifier = 0.50;
        public static int joystickPort = 0;
        public static int xboxControllerPort = 1;
        public static int streamDeckPort = 2;
        public static double[] joystickDeadzones = {0.1, 0.1, 0.15};
    }
    public static class DriveConstants {
        public static Constraints turnControllerConstraints = new Constraints(TeleoperatedConstants.maxAngularVelocity, TeleoperatedConstants.maxAngularVelocity * 2);
        public static double turnCommandP = 2; //
        public static double turnCommandI = 2.5;
        public static double turnCommandD = 0.0;
        public static double turnCommandAngleTolerance = Math.toRadians(3);
        public static double turnCommandVelocityTolerance = turnCommandAngleTolerance / 2;
        public static double swerveDriveFFkS = 0.0875; // overcoming static friction
        public static double swerveDriveFFkV = 0; // not needed
        public static double swerveDriveFFkA = 0; // not needed
        public static double drivingGearRatio = 8.14;
        public static double turningGearRatio = 12.8;
        public static double wheelDiameter = 0.1016;
        public static class AutoPiecePickup {
            public static Constraints turnControllerConstraints = new Constraints(TeleoperatedConstants.maxAngularVelocity, TeleoperatedConstants.maxAngularVelocity * 2); // these values are definitely wrong for vision lmao
            public static double turnCommandP = 4;
            public static double turnCommandI = 3.5;
            public static double turnCommandD = 0;
            public static double turnCommandAngleTolerance = Math.toRadians(0.375);
            public static double turnCommandVelocityTolerance = turnCommandAngleTolerance / 2;
            public static double piecePickupVelocity = 2; // 2 meters per second
            public static double gamePieceXValue = 7.11835;
            public static double[] gamePieceYValues = {0.919226, 2.138426, 3.357626, 4.576826};
        }
        public static class PlacementOffsets {
            public static double highConeOffset = 1.397;
            public static double highCubeOffset = 1.7018;
            public static double midConeOffset = 1.45;
            public static double midCubeOffset = 1.7018;
            public static double lowConeOffset = 1.5494;
            public static double lowCubeOffset = 1.5494;
        }
    }
    public static class DEBUGGING {
        public static boolean useDebugTab = false;
        public static boolean reportSwervePositions = false;
        public static boolean reportChassisSpeeds = false;
        public static boolean bypassAutoChecks = false; // VERY DANGEROUS TO LEAVE TRUE
    }

    public static class AutonomousConstants {
        public static double maxVelocity = 2;
        public static double maxAccel = 3;
        public static double lowestVelocity = 0.05;
        public static double lowestAngularVelocity = 0.1;
        public static double translationKP = 5; // 5
        public static double translationKI = 0;
        public static double translationKD = 0;
        public static double rotationKP = 1; // 3
        public static double rotationKI = 0;
        public static double rotationKD = 0;
        public static boolean usePPServer = true;
        public static boolean usePoseReset = false;
        public static boolean usePhotonCamera = false;
        public static boolean useLLForPoseEstimation = true;
        public static String photonCameraName = "photoncameraone";
        public static Transform3d cameraTransformation = new Transform3d(new Translation3d(-0.36195, 0.0, -0.5969), new Rotation3d());
        public static double chargeInitialSpeed = 2.25;
        public static double chargeFinalSpeed = 0.5;
        public static double chargeDAngleThreshold = -17.5;
        public static PathConstraints onTheFlyConstraints = new PathConstraints(2, 2);
        public static PIDConstants translationConstants = new PIDConstants(translationKP, translationKI, translationKD);
        public static PIDConstants rotationConstants = new PIDConstants(rotationKP, rotationKI, rotationKD);
    }
    public static class MandibleConstants {
        public static int forwardChannel = 5;
        public static int reverseChannel = 4;
        public static int leftMotorID = 10;
        public static int rightMotorID = 11;
        public static double outtakeSpeed = 0.35;
        public static double intakeSpeed = 0.50;
        public static double idleSpeed = 0.20;
        public static double defaultOuttakeTime = 0.50;
        public static boolean useDistanceSensor = true;
        public static int distanceSensorID = 12;
        public static double distanceRangeThreshold = 40; // 40 millimeters
    }
    public static class ArmConstants {
        public static double armIncrement = 0.5;
        public static double armSlowdownFactor = 0.2;
        public static int falconID = 9;
        public static int limitSwitchID = 0;
        public static double manualControlDeadzone = 0.1;
        public static double autoPlacementTolerance = 0.1;
        public static HashMap<String, Double> armValues = new HashMap<String, Double>();
        static {
            armValues.put("FullyRetracted", 0.0);
            armValues.put("Substation", 37.9);
            armValues.put("Floor", 42.1);
            armValues.put("BottomPlacement", 40.4);
            armValues.put("HighConePrep", 35.2);
            armValues.put("HighConePlacement", 37.16);
            armValues.put("MidConePrep", 35.2);
            armValues.put("MidConePlacement", 38.5);
            armValues.put("HighCube", 37.1);
            armValues.put("MidCube", 38.54);
            armValues.put("Optimized", 21.5);
            armValues.put("FloorPickupPrep", 40.0);
        }
    }
}
