package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule implements Sendable {
    private final TalonFX driveTalonFX;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final DutyCycleEncoder directionDutyCycle;

    private final boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;

    private final int driveid;
    private final String encoderOffsetKey;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) { // removed args: double absoluteEncoderOffset

        driveid = driveMotorId;

        encoderOffsetKey = "absoluteEndcoderOffsetRadWheel"+driveid;
        Preferences.initDouble(encoderOffsetKey, 0);

        //this.absoluteEncoderOffsetRad = 0; //set in preferences
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
        turningPidController.enableContinuousInput(0, 2 * Math.PI);
        loadPreferences(); //to set initial values from storage
        resetEncoders();
    }

    public void loadPreferences(){
        this.absoluteEncoderOffsetRad = Preferences.getDouble(encoderOffsetKey, 0);
    }

    public double getDrivePosition() {
        //return driveEncoder.getPosition();
        //return driveTalonFX.getSelectedSensorPosition() / 2048;

        return (driveTalonFX.getSelectedSensorPosition() / 2048)
            * ModuleConstants.kDriveEncoderRot2Meter;


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
        return (driveTalonFX.getSelectedSensorVelocity() / 2048) 
         * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
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
        turningEncoder.setPosition(getAbsoluteEncoderRad()); //TODO: this isn't referencing the right encoder. now using directionDutyCycle
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            driveTalonFX.set(ControlMode.PercentOutput, 0);
            return;
        }
        driveTalonFX.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public void stop() {
        driveTalonFX.set(ControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.addDoubleProperty("Drive Percentage", () -> driveTalonFX.getMotorOutputPercent() , null);
        builder.addDoubleProperty("Rotate Percentage", () -> turningMotor.getAppliedOutput() , null);
        builder.addDoubleProperty("RotationRad", () -> getTurningPosition(), null);
    }
}