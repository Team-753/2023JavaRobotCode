package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {
    public final int driveMotorID;
    public final int turnMotorID;
    public final int analogID;
    public final double analogOffset;
    public final Rotation2d xAngle;
    public final String name;

    public SwerveModuleConfig(int driveID, int turnID, int absoluteID, double absoluteOffset, double XAngle, String Name) {
        driveMotorID = driveID;
        turnMotorID = turnID;
        analogID = absoluteID;
        analogOffset = absoluteOffset;
        xAngle = Rotation2d.fromDegrees(XAngle);
        name = Name;
    }
}
