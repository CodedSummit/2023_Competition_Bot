// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmMoveToPosition;
import frc.robot.commands.Autos;
import frc.robot.commands.CalibrateArmCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SwerveBalanceFwdBack;
import frc.robot.commands.SwerveFixedMoveCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveXPark;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.ArmDistanceSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PIDArmSubsystem armSubsystem = new PIDArmSubsystem();
  private final ArmDistanceSubsystem armDistanceSubsystem = new ArmDistanceSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private SwerveJoystickCmd swerveJoystickCmd;

  SendableChooser<Command> m_auto_chooser;

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
 
    swerveJoystickCmd = new SwerveJoystickCmd(
      swerveSubsystem,
      m_driverController,
      armSubsystem,
      armDistanceSubsystem);
    swerveSubsystem.setDefaultCommand(swerveJoystickCmd); 

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    armTab.add("Arm", armSubsystem)
        .withSize(2,2);
    armTab.add("Intake", intakeSubsystem)
        .withSize(2,2);
    armTab.add(new CalibrateArmCommand(armSubsystem));
    armTab.add("Distance", armDistanceSubsystem);

    ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

    swerveTab.add(new SwerveXPark(swerveSubsystem));

    SmartDashboard.putData("Balance", new SwerveBalanceFwdBack(swerveSubsystem));
    SmartDashboard.putData(swerveSubsystem);

//TODO: calibrate subsystem on robot start.

    configureAuto();

    // Configure the trigger bindings
    configureBindings();
  }

  public void runStartupCalibration(){
    if(!armSubsystem.isCalibrated()){
      new CalibrateArmCommand(armSubsystem).schedule();
    }
  }

  public void loadPreferences(){
    swerveSubsystem.loadPreferences();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
     //   .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    // Left Bumper controls field orientation for drive mode. Upressed (default) is field oriented
    //     Pressed is robot oriented
    m_driverController.leftBumper()
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(false)))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(true)));

    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getDampenedSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

    m_driverController.axisGreaterThan(3, 0.5)
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getTurboSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

    m_driverController.b()
      .onTrue(new InstantCommand(() -> intakeSubsystem.InputIn()))
      .onFalse(new InstantCommand(() -> intakeSubsystem.stop()));

    m_driverController.x()
      .onTrue(new InstantCommand(() -> intakeSubsystem.InputOut()))
      .onFalse(new InstantCommand(() -> intakeSubsystem.stop()));

      m_driverController.povUp()
      .onTrue(new InstantCommand(() -> armSubsystem.Up()))
      .onFalse(new InstantCommand(() -> armSubsystem.stop()));

      m_driverController.povDown()
      .onTrue(new InstantCommand(() -> armSubsystem.Down()))
      .onFalse(new InstantCommand(() -> armSubsystem.stop()));

      m_driverController.povLeft()
      .onTrue(new InstantCommand(() -> armSubsystem.setGoal(344)));

      m_driverController.povRight()
      .onTrue(new InstantCommand(() -> armSubsystem.setGoal(10)));

      m_driverController.a()
      .onTrue(new SwerveXPark(swerveSubsystem));

  }

  private void configureAuto(){

      Command utahAuto = Autos.UtahAuto(armSubsystem, swerveSubsystem, intakeSubsystem);

      Command balancePortion = Autos.BalancePortion(swerveSubsystem);

  // A chooser for autonomous commands
    m_auto_chooser = new SendableChooser<>();

    m_auto_chooser.setDefaultOption("Utah Auto", utahAuto);
    m_auto_chooser.addOption("Balance Portion", balancePortion);

    SmartDashboard.putData("Auto Routine", m_auto_chooser);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new InstantCommand();
    return m_auto_chooser.getSelected();

    //return Autos.UtahAuto(armSubsystem, swerveSubsystem, intakeSubsystem);

    

    //return new SwerveFixedMoveCmd(swerveSubsystem, 0.0, -0.3, 10.0);
    /*// 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics);

// 2. Generate trajectory
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
              //new Translation2d(1, 0),
              //new Translation2d(1, -1)
              ),
      new Pose2d(2, -1, Rotation2d.fromDegrees(0)), //angle was 180
      trajectoryConfig);

// 3. Define PID controllers for tracking trajectory
PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(0, 2*Math.PI);

// 4. Construct command to follow trajectory
SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerveSubsystem::getPose,
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

// 5. Add some init and wrap-up, and return everything
return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules()));

      */
}
  
}
