package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANCoder encoder;
    private final PIDController anglePID;

    private final double encoderOffset;

    public SwerveModule(int driveID, int turnID, int encoderID, double encoderOffset) {
        this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        this.encoder = new CANCoder(encoderID);
        this.encoderOffset = encoderOffset;

        this.anglePID = new PIDController(0.5, 0.0, 0.0); // Tune PID values as needed
        this.anglePID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoder();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to minimize wheel rotation
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState, Rotation2d.fromDegrees(encoder.getAbsolutePosition())
        );

        // Set drive speed
        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveSubsystem.MAX_SPEED);

        // Set turning angle
        double targetAngle = optimizedState.angle.getRadians();
        double currentAngle = Math.toRadians(encoder.getAbsolutePosition());
        double angleOutput = anglePID.calculate(currentAngle, targetAngle);

        turnMotor.set(angleOutput);
    }

    public SwerveModuleState getState() {
        double speed = driveMotor.getEncoder().getVelocity(); // Assuming velocity in meters/second
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        return new SwerveModuleState(speed, Rotation2d.fromRadians(angle));
    }

    public void resetEncoder() {
        encoder.setPositionToAbsolute();
        encoder.configMagnetOffset(encoderOffset);
    }
}