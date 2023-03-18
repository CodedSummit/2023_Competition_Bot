package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveXPark;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            //DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            //DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            //DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            //DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    //private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());

    private double yTiltOffset;

    private GenericEntry turboSpeedFactor;
    private GenericEntry normalSpeedFactor;
    private GenericEntry dampenedSpeedFactor;


    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                zeroYTilt();
            } catch (Exception e) {
            }
        }).start();
        loadPreferences();

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.add("Front Left", frontLeft)
            .withSize(2,2);
        swerveTab.add("Front Right", frontRight)
            .withSize(2,2);
        swerveTab.add("Back Right", backRight)
            .withSize(2,2)
            .withPosition(0, 2);
        swerveTab.add("Back Left", backLeft)
            .withSize(2,2)
            .withPosition(2, 2);

        turboSpeedFactor = swerveTab.add("Turbo Percentage", 0.9)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();

        normalSpeedFactor = swerveTab.add("Normal Percentage", .5)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();
 
        dampenedSpeedFactor = swerveTab.add("Dampened Percentage", .2)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();
    }

    public double getTurboSpeedFactor(){
        return turboSpeedFactor.getDouble(1.0);
    }
    public double getNormalSpeedFactor(){
        return normalSpeedFactor.getDouble(0.5);
    }
    public double getDampenedSpeedFactor(){
        return dampenedSpeedFactor.getDouble(0.2);
    }


    public void loadPreferences(){
        frontLeft.loadPreferences();
        frontRight.loadPreferences();
        backLeft.loadPreferences();
        backRight.loadPreferences();
    }

    public void zeroHeading() {
        gyro.reset();
        
    }

    public double xTilt() {
        return gyro.getXFilteredAccelAngle();
    }
    public double yTilt(){ //front back
        //15 degrees max
        double yAngle = gyro.getYFilteredAccelAngle() - yTiltOffset;
        //convert to negative number of tilted the other way
        if(yAngle > 180){
            yAngle -= 360;
        }
        return yAngle;

    }
    public void zeroYTilt(){
        yTiltOffset = gyro.getYFilteredAccelAngle();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360); 
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    private SwerveModulePosition [] getModulePositions(){
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
            backRight.getPosition()};
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        //SmartDashboard.putNumber("X Tilt", xTilt());
        SmartDashboard.putNumber("Y Tilt", yTilt());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);

    }
}