package frc.FSLib.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;

@Builder
public record SwerveModuleConstants(int driveMotorId, int steerMotorId, int cancoderId, double cancoderOffset, Translation2d modulePosition) {
}
