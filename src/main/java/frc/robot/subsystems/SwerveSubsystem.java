package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        kinematics = new SwerveDriveKinematics(
            Constants.SwerveConstants.FRONT_LEFT_LOCATION,
            Constants.SwerveConstants.FRONT_RIGHT_LOCATION,
            Constants.SwerveConstants.BACK_LEFT_LOCATION,
            Constants.SwerveConstants.BACK_RIGHT_LOCATION
        );

        swerveModules = new SwerveModule[] {
            new SwerveModule(
                Constants.SwerveConstants.FRONT_LEFT_DRIVE_ID,
                Constants.SwerveConstants.FRONT_LEFT_TURN_ID,
                Constants.SwerveConstants.FRONT_LEFT_ENCODER_ID,
                Constants.SwerveConstants.FRONT_LEFT_ENCODER_OFFSET
            ),
            new SwerveModule(
                Constants.SwerveConstants.FRONT_RIGHT_DRIVE_ID,
                Constants.SwerveConstants.FRONT_RIGHT_TURN_ID,
                Constants.SwerveConstants.FRONT_RIGHT_ENCODER_ID,
                Constants.SwerveConstants.FRONT_RIGHT_ENCODER_OFFSET
            ),
            new SwerveModule(
                Constants.SwerveConstants.BACK_LEFT_DRIVE_ID,
                Constants.SwerveConstants.BACK_LEFT_TURN_ID,
                Constants.SwerveConstants.BACK_LEFT_ENCODER_ID,
                Constants.SwerveConstants.BACK_LEFT_ENCODER_OFFSET
            ),
            new SwerveModule(
                Constants.SwerveConstants.BACK_RIGHT_DRIVE_ID,
                Constants.SwerveConstants.BACK_RIGHT_TURN_ID,
                Constants.SwerveConstants.BACK_RIGHT_ENCODER_ID,
                Constants.SwerveConstants.BACK_RIGHT_ENCODER_OFFSET
            )
        };

        odometry = new SwerveDriveOdometry(
        kinematics,
        new Rotation2d(),
        new SwerveModulePosition[] {
        swerveModules[0].getPosition(),
        swerveModules[1].getPosition(),
        swerveModules[2].getPosition(),
        swerveModules[3].getPosition()
    });
    }

    public void drive(double vx, double vy, double omega) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.MAX_SPEED);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(0), // Replace with gyro value
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            });
    }
}
