// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InputSystemConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax intakeMotor;
  private double intakespeed, outtakespeed;

  public IntakeSubsystem() {

    intakeMotor = new CANSparkMax(InputSystemConstants.kInputMotorCANid, MotorType.kBrushed);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakespeed = 0.65;
    outtakespeed = 0.65;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void InputIn(){
    intakeMotor.set(-intakespeed);
  }

  public void InputOut(){
    intakeMotor.set(outtakespeed);
  
  }

  public void stop(){
    intakeMotor.stopMotor();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    builder.addDoubleProperty("Motor", () -> intakeMotor.get() , null);
    builder.addDoubleProperty("intake speed", () -> intakespeed , (n) -> intakespeed = n);
    builder.addDoubleProperty("outtake speed", () -> intakespeed , (n) -> outtakespeed = n);

}

}
