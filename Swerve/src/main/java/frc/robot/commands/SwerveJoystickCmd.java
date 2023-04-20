package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmDistanceSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final PIDArmSubsystem armSubsystem;
    private final ArmDistanceSubsystem armDistanceSubsystem;

    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private boolean fieldOriented;
    private double motionScale;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, CommandXboxController m_driverController, PIDArmSubsystem armSubsystem, ArmDistanceSubsystem armDistanceSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.armSubsystem = armSubsystem;
        this.armDistanceSubsystem = armDistanceSubsystem;
        this.xSpdFunction = () -> -m_driverController.getLeftY();
        this.ySpdFunction = () -> -m_driverController.getLeftX();
        this.turningSpdFunction = () -> -m_driverController.getRightX();
        this.fieldOriented = true;
        this.motionScale = swerveSubsystem.getNormalSpeedFactor();
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Calcualte motionscale based on arm up and proximity close
        //check ignore status?
        double effectiveMotionScale = this.motionScale;
        if(armSubsystem.getPosition() > 100 && armDistanceSubsystem.distance_cm() < 250){
            effectiveMotionScale = swerveSubsystem.getDampenedSpeedFactor();
        }
        
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get() * effectiveMotionScale;
        double ySpeed = ySpdFunction.get() * effectiveMotionScale;
        double turningSpeed = turningSpdFunction.get() * effectiveMotionScale;

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

    public void setMotionScale(double d) {
        this.motionScale = d;
    }
}