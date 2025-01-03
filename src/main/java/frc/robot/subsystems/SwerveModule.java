package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANcoder encoder;
    private final PIDController anglePID;

    private final double encoderOffset;

    public SwerveModule(int driveID, int turnID, int encoderID, double encoderOffset) {
        this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        this.encoder = new CANcoder(encoderID);
        this.encoderOffset = encoderOffset;

        this.anglePID = new PIDController(0.5, 0.0, 0.0); // Tune PID values as needed
        this.anglePID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoder();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Calculate the adjusted current angle
        double rawAngle = encoder.getAbsolutePosition().getValue();
        double adjustedAngle = (rawAngle - encoderOffset + 360) % 360; // Adjust for offset and wrap to [0, 360)
    
        // Optimize the state to minimize wheel rotation
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState, Rotation2d.fromDegrees(adjustedAngle)
        );
    
        // Set drive speed
        driveMotor.set(optimizedState.speedMetersPerSecond / Constants.SwerveConstants.MAX_SPEED);
    
        // Set turning angle
        double targetAngle = optimizedState.angle.getRadians();
        double currentAngle = Math.toRadians(adjustedAngle);
        double angleOutput = anglePID.calculate(currentAngle, targetAngle);
    
        turnMotor.set(angleOutput);
    }

    public SwerveModuleState getState() {
        double speed = driveMotor.getEncoder().getVelocity(); // Assuming velocity in meters/second
    
        // Manually calculate the adjusted angle with the offset
        double rawAngle = encoder.getAbsolutePosition().getValue();
        double adjustedAngle = (rawAngle - encoderOffset + 360) % 360; // Adjust for offset and wrap to [0, 360)
    
        return new SwerveModuleState(speed, Rotation2d.fromDegrees(adjustedAngle));
    }

    public void resetEncoder() {
        encoder.setPosition(encoder.getAbsolutePosition().getValue()); // Set position to absolute value
    }

    public SwerveModulePosition getPosition() {
        double distance = driveMotor.getEncoder().getPosition(); // Replace with actual distance retrieval method
        double angle = Math.toRadians(encoder.getAbsolutePosition().getValue());
        return new SwerveModulePosition(distance, Rotation2d.fromRadians(angle));
    }
}
