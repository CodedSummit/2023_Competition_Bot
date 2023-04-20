// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class CalibrateArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PIDArmSubsystem armSubsystem;

  private boolean lowerLimitFinalState;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateArmCommand(PIDArmSubsystem m_armSubsystem) {
    armSubsystem = m_armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //move arm
    armSubsystem.disable(); //disable PID
    if(armSubsystem.atLowerLimit()){
      lowerLimitFinalState = false;
      armSubsystem.RawUp(0.1);
    } else {
      lowerLimitFinalState = true;
      armSubsystem.RawDown(0.02);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops arm
    armSubsystem.hardStop();
    if(!interrupted){
      armSubsystem.resetEncoder();
      //Turn PID back on
      armSubsystem.enable();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //test limit switch
    return armSubsystem.atLowerLimit() == lowerLimitFinalState;
  }
}
