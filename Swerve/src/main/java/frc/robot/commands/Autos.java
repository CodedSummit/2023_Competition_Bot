// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
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

  public static CommandBase BoiseMobility(PIDArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
      new CalibrateArmCommand(armSubsystem),
      new CalibrateArmCommand(armSubsystem),
      new ArmMoveToPosition(armSubsystem, 323),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, -0.25, 1.5),
      new IntakeCommand(intakeSubsystem, 1),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, 0.25, 1.0),
      new ArmMoveToPosition(armSubsystem, 5),
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, .25, 1)
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
      new SwerveFixedMoveCmd(swerveSubsystem, 0.0, .25, 1)
    );
  }

  public static CommandBase BalancePortion(SwerveSubsystem swerveSubsystem){
    return new SequentialCommandGroup(
      new SwerveFixedMoveTillTiltCmd(swerveSubsystem,0 , .3, 2),
      new SwerveBalanceFwdBack(swerveSubsystem)
    );
  }

  private Autos() {
    //throw new UnsupportedOperationException("This is a utility class!");
  }
}
