// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase UtahAuto(PIDArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
      new CalibrateArmCommand(armSubsystem),
      new CalibrateArmCommand(armSubsystem),
      new ArmMoveToPosition(armSubsystem, 323),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, -0.4, 1.7),
      new IntakeCommand(intakeSubsystem, 1),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, 0.4, 1.0),
      new ArmMoveToPosition(armSubsystem, 20),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, 1, 1.4),
      new SwerveFixedMoveCmd(swerveSubsystem, 1, 0, .1)

    );
  }

  public static CommandBase OnlyBalance(SwerveSubsystem swerveSubsystem){
    return new SequentialCommandGroup(
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, .2, 1.0),
      //new SwerveFixedMoveTillTiltCmd(swerveSubsystem, 0, .5, 3),
      new SwerveBalanceFwdBack(swerveSubsystem)

    );
  }

  public static CommandBase BoiseBalance(PIDArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
      new CalibrateArmCommand(armSubsystem),
      new CalibrateArmCommand(armSubsystem),
      new ArmMoveToPosition(armSubsystem, 323),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, -0.25, 1.5),
      new IntakeCommand(intakeSubsystem, 1),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, 0.25, 1.0),
      new ArmMoveToPosition(armSubsystem, 5),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, .5, 1.0),
      //new SwerveFixedMoveTillTiltCmd(swerveSubsystem, 0, .5, 3),
      new SwerveBalanceFwdBack(swerveSubsystem)

    );
  }

  public static CommandBase BoiseMobilityShiftRight(PIDArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
      new CalibrateArmCommand(armSubsystem),
      new CalibrateArmCommand(armSubsystem),
      new ArmMoveToPosition(armSubsystem, 68.5),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, -0.25, 1.5),
      new IntakeCommand(intakeSubsystem, 1),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, 0.25, 1.0),
      new ArmMoveToPosition(armSubsystem, 0),
      new SwerveMoveRight(swerveSubsystem, 0.1, 0.2), //4 inches
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, .25, 1.7)
    );
  }

  public static CommandBase BoiseMobilityShiftLeft(PIDArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
      new CalibrateArmCommand(armSubsystem),
      new CalibrateArmCommand(armSubsystem),
      new ArmMoveToPosition(armSubsystem, 68.5),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, -0.25, 1.5),
      new IntakeCommand(intakeSubsystem, 1),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, 0.25, 1.0),
      new ArmMoveToPosition(armSubsystem, 0),
      new SwerveMoveLeft(swerveSubsystem, 0.1, 0.2), //4 inches
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, .25, 1.7)
    );
  }

  public static CommandBase BoiseCubeEject(IntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
      new EjectCubeCommand(intakeSubsystem, 1)
    );
  }

  public static CommandBase MobilityShiftRight(PIDArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
      new CalibrateArmCommand(armSubsystem),
      new CalibrateArmCommand(armSubsystem),
      new ArmMoveToPosition(armSubsystem, 323),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, -0.25, 1.5),
      new IntakeCommand(intakeSubsystem, 1),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, 0.25, 1.0),
      new ArmMoveToPosition(armSubsystem, 5),
      new SwerveFixedMoveCmd(swerveSubsystem, .25, 0, 1.5),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, .25, 1.7)
    );
  }

  public static CommandBase BalancePortion(SwerveSubsystem swerveSubsystem){
    return new SequentialCommandGroup(
      new SwerveFixedMoveTillTiltCmd(swerveSubsystem,0 , .3, 2), //positive y is backward
      new SwerveBalanceFwdBack(swerveSubsystem)
    );
  }

  public static CommandBase EjectCubeBalance(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem){
    return new SequentialCommandGroup(
      new EjectCubeCommand(intakeSubsystem, 1),
      new SwerveFixedMoveTillTiltCmd(swerveSubsystem,0 , .5, 2),
      new SwerveBalanceFwdBack(swerveSubsystem)
    );
  }

  public static CommandBase EjectCubeMobilityBalance(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem){
    return new SequentialCommandGroup(
      new EjectCubeCommand(intakeSubsystem, 1),
      new SwerveMoveBackward(swerveSubsystem, .5, 4),
      new SwerveFixedMoveTillTiltCmd(swerveSubsystem,0 , -.5, 2),
      new SwerveBalanceFwdBack(swerveSubsystem)
    );
  }

  public static CommandBase DistanceMove(SwerveSubsystem swerveSubsystem){
    return new SequentialCommandGroup(
      new SwerveMoveForward(swerveSubsystem, 0.1, 0.1), //2 inches
      new SwerveMoveRight(swerveSubsystem, 0.1, 0.1), //2 inches
      new SwerveMoveBackward(swerveSubsystem, 0.1, 0.1), //2 inches
      new SwerveMoveLeft(swerveSubsystem, 0.1, 0.1) //2 inches
    );
  }

  private Autos() {
    //throw new UnsupportedOperationException("This is a utility class!");
  }
}
