package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveXPark extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;

    double timeoutExpires;

    public SwerveXPark(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = {
            new SwerveModuleState(0, new Rotation2d(45)),
            new SwerveModuleState(0, new Rotation2d(225)),
            new SwerveModuleState(0, new Rotation2d(45)),
            new SwerveModuleState(0, new Rotation2d(225)),
        };

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        //swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}