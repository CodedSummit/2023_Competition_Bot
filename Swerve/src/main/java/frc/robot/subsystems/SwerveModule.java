package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

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

    //private final CANEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;
    private final SparkMaxAnalogSensor directionEncoder;

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

        //get absolute encoder attached to turning motor
        directionEncoder = turningMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

        driveTalonFX.configFactoryDefault();

        driveTalonFX.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        //driveEncoder = driveTalonFX.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
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
        SmartDashboard.putNumber("Wheel-"+driveid, turningEncoder.getPosition());
        return turningEncoder.getPosition();
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
        double angle = directionEncoder.getPosition();
        angle -= absoluteEncoderOffsetRad;
        
        //double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        //angle *= 2.0 * Math.PI;

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
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        //driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        driveTalonFX.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + driveid + "] state", state.toString());
        SmartDashboard.putNumber("Swerve[" + driveid + "] turn existing", getTurningPosition());
        SmartDashboard.putNumber("Swerve[" + driveid + "] turn desired", state.angle.getRadians());
    }

    public void stop() {
        driveTalonFX.set(ControlMode.PercentOutput, 0);
        //driveMotor.set(0);
        turningMotor.set(0);
    }
}