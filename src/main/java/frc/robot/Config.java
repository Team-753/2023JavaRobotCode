package frc.robot;

import java.util.HashMap;

import frc.robot.configs.SwerveModuleConfig;

public class Config {
    public static class DimensionalConstants {
        public static double wheelBase = 0.5969;
        public static double trackWidth = 0.4954524;
        public static HashMap<String, SwerveModuleConfig> SwerveModuleConfigurations = new HashMap<String, SwerveModuleConfig>();
        static {
            SwerveModuleConfigurations.put("frontLeftModule", new SwerveModuleConfig(1, 2, 0, 76, -135, "frontLeftModule"));
            SwerveModuleConfigurations.put("frontRightModule", new SwerveModuleConfig(3, 4, 1, 309, 135, "frontRightModule"));
            SwerveModuleConfigurations.put("rearRightModule", new SwerveModuleConfig(5, 6, 2, 227, 45, "rearLeftModule"));
            SwerveModuleConfigurations.put("rearLeftModule", new SwerveModuleConfig(7, 8, 3, 22.6, -45, "rearRightModule"));
        }
        public static String limelightName = "limelight";
        public static int pcmID = 61;
        public static double apriltagThresholdDistance = 1.5;
        public static double fieldHeight = 8.0137;
        public static double fieldLength = 16.54175;
    }
    public static class TeleoperatedConstants {
        public static double maxVelocity = 4.25;
        public static double maxAngularVelocity = maxVelocity / Math.hypot(DimensionalConstants.trackWidth / 2, DimensionalConstants.wheelBase / 2);
        public static double turningSpeedModifier = 0.60;
        public static double toggleableSpeedModifier = 0.50;
        public static int joystickPort = 0;
        public static int xboxControllerPort = 1;
        public static int streamDeckPort = 2;
        public static double[] joystickDeadzones = {0.1, 0.1, 0.15};
    }
    public static class AutonomousConstants {
        public static double maxVelocity = 2;
        public static double maxAccel = 4;
        public static double lowestVelocity = 0.05;
        public static double lowestAngularVelocity = 0.1;
        public static double translationKP = 5;
        public static double translationKI = 0;
        public static double translationKD = 0;
        public static double rotationKP = 1;
        public static double rotationKI = 0;
        public static double rotationKD = 0;
        public static boolean usePPServer = true;
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
            armValues.put("Floor", 41.75);
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
