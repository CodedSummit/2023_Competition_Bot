// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSystemConstants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax armMotor;

  private DigitalInput upperLimitSwitch;
  private DigitalInput lowerLimitSwitch;

  private RelativeEncoder armEncoder;

  private SlewRateLimiter limiter;
  private Double target;

  public ArmSubsystem() {

    armMotor = new CANSparkMax(ArmSystemConstants.kArmMotorCANid, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armEncoder = armMotor.getEncoder();

    limiter = new SlewRateLimiter(0.5, -2, 0);
    target = 0.0;
    
  upperLimitSwitch = new DigitalInput(ArmSystemConstants.kUpperLimitSwitchPort);

  lowerLimitSwitch = new DigitalInput(ArmSystemConstants.kLowerLimitSwitchPort);
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

  public boolean atUpperLimit(){
    return upperLimitSwitch.get(); ///.get() > 0;
  }
  public boolean atLowerLimit(){
    return lowerLimitSwitch.get(); //Counter.get() > 0;
  }

  public void resetEncoder(){
    armEncoder.setPosition(0);
  }

  public double getPosition(){
    return armEncoder.getPosition();
  }

  public void Up(){
    if(!atUpperLimit()){
      target = 0.75;
      //armMotor.set(0.25);
    }
  }

  public void Down(){
    if(!atLowerLimit()){
      target = -0.25;
      //armMotor.set(-0.1);
    }
  }

  public void stop(){
    target = 0.0;
  }

  public void hardStop(){
    armMotor.stopMotor();
    limiter.reset(0);

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
    if(atUpperLimit() && armMotor.get() > 0){
      hardStop();
    }
    if(atLowerLimit() && armMotor.get() < 0){
      hardStop();
    }

    armMotor.set(limiter.calculate(target));
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      builder.addDoubleProperty("Power", () -> armMotor.get() , null);
      builder.addDoubleProperty("Position", () -> getPosition() , null);
      builder.addBooleanProperty("UpperLimit", () -> atUpperLimit() , null);
      builder.addBooleanProperty("LowerLimit", () -> atLowerLimit(), null);
  }

}
