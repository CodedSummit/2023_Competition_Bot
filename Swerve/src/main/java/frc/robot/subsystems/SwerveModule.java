package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {

    private final TalonFX driveTalonFX;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final DutyCycleEncoder directionDutyCycle;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final int driveid;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        driveid = driveMotorId;
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        //absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveTalonFX = new TalonFX(driveMotorId, "rio");
        //driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        //use the same DIO pins as motor numbering. 
        // TODO: add this as configuration
        directionDutyCycle = new DutyCycleEncoder(driveMotorId);
        directionDutyCycle.setDutyCycleRange(1/4096, 4096/4096);

        driveTalonFX.configFactoryDefault();

        driveTalonFX.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningEncoder = turningMotor.getEncoder();

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        //return driveEncoder.getPosition();
        return driveTalonFX.getSelectedSensorPosition();
    }

    public double getTurningPosition() {
        SmartDashboard.putNumber("Wheel-"+driveid, getAbsoluteEncoderRad());
        return getAbsoluteEncoderRad();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.getDrivePosition(), new Rotation2d(this.getTurningPosition()));
      }

    public double getDriveVelocity() {
        //return driveEncoder.getVelocity();
        return driveTalonFX.getSelectedSensorVelocity(); // / 2048 * Units.millisecondsToSeconds(100)
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        directionDutyCycle.getAbsolutePosition();
        double angle = directionDutyCycle.getAbsolutePosition();
        
        angle *= 2.0 * Math.PI; //convert 0-1 range into radians

        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        //driveEncoder.setPosition(0);
        driveTalonFX.setSelectedSensorPosition(0.0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveTalonFX.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveTalonFX.set(ControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }
}