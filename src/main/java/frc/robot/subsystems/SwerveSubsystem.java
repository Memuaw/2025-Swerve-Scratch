package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Pigeon2 gyro;

    public SwerveSubsystem() {
        kinematics = new SwerveDriveKinematics(
            Constants.SwerveConstants.FRONT_LEFT_LOCATION,
            Constants.SwerveConstants.FRONT_RIGHT_LOCATION,
            Constants.SwerveConstants.BACK_LEFT_LOCATION,
            Constants.SwerveConstants.BACK_RIGHT_LOCATION
        );

        gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_ID);
        gyro.setYaw(0);

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
            Rotation2d.fromDegrees(gyro.getYaw().getValue()), // Use gyro heading
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            }
        );
    }

    public void drive(double vx, double vy, double omega) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.MAX_SPEED);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }
    
    public void zeroGyro() {
        gyro.setYaw(0); // Reset the gyro yaw to zero
    }

    @Override
    public void periodic() {
        // Get the gyro heading as a Rotation2d object
        Rotation2d heading = Rotation2d.fromDegrees(gyro.getYaw().getValue());

        // Update the odometry using the heading and module positions
        odometry.update(
            heading,
            new SwerveModulePosition[] {
               swerveModules[0].getPosition(),
               swerveModules[1].getPosition(),
               swerveModules[2].getPosition(),
               swerveModules[3].getPosition()
            }
        );

        // Report telemetry
        Pose2d pose = odometry.getPoseMeters();
        SmartDashboard.putNumber("Pose X", pose.getX());
        SmartDashboard.putNumber("Pose Y", pose.getY());
        SmartDashboard.putNumber("Pose Heading", pose.getRotation().getDegrees());

        SmartDashboard.putNumber("Gyro Heading", gyro.getYaw().getValue());

        for (SwerveModule module : swerveModules) {
            module.reportTelemetry();
        }
    }
}
