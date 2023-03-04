// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmMoveToPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;

  private boolean lowerLimitFinalState;
  private double target_position;
  private boolean moving_up;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMoveToPosition(ArmSubsystem m_armSubsystem, double arm_position) {
    armSubsystem = m_armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    target_position = arm_position;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //move arm
    if(armSubsystem.getPosition() > target_position){
      armSubsystem.Down(0.75);
      moving_up = false;
    } else {
      armSubsystem.Up(1);
      moving_up = true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops arm
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //test limit switch
    if(moving_up){
      return armSubsystem.getPosition() >= target_position;
    } else {
      return armSubsystem.getPosition() <= target_position;
    }
  }
}
