package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveBalanceFwdBack extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private boolean fieldOriented;
    private double runSeconds;
    double timeoutExpires;
    private double drivePower, drivePowerLowerLimit, signChangeReduction;
    private double lastAngle;

    public SwerveBalanceFwdBack(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = () -> 0.0;
        this.ySpdFunction = () -> 0.0;
        this.turningSpdFunction = () -> 0.0;
        this.fieldOriented = false;
        //this.runSeconds = run_seconds;
        this.drivePower = 0.25;
        this.signChangeReduction = 0.05;
        this.drivePowerLowerLimit = 0.05;
        this.lastAngle = -1;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    
    }

    @Override
    public void initialize() {
        timeoutExpires = Timer.getFPGATimestamp() + runSeconds;
        this.drivePower = 0.20;
        this.signChangeReduction = 0.05;
        this.drivePowerLowerLimit = 0.05;
        this.lastAngle = -1;
    }

    @Override
    public void execute() {
        //decide fwd back based on gyro
        double angle = swerveSubsystem.yTilt();

        //detect sign change
        if(Math.abs(angle) > 3){
            if(lastAngle>0 && angle<0 && drivePower > drivePowerLowerLimit){
                drivePower -= signChangeReduction;
            } else if (lastAngle<0 && angle>0 && drivePower > drivePowerLowerLimit){
                drivePower -= signChangeReduction;
            }
            lastAngle = angle;
        }

        double xSpeed = 0;
        double ySpeed = 0;
        if(Math.abs(angle) < 3 ){
            //balanced. do nothing, or lock wheels?
            xSpeed = 0;
            
        } else if(angle > 0){
            //drive fwd?
            xSpeed = drivePower;
        } else if(angle < 0){
            //drive backward?
            xSpeed = -drivePower;
        }



        
        
        // 1. Get real-time joystick inputs
        //double xSpeed = xSpdFunction.get();
        //double ySpeed = ySpdFunction.get();
        double turningSpeed = 0; //turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (this.fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    public void setFieldOriented(boolean fieldOriented){
        this.fieldOriented = fieldOriented;
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}