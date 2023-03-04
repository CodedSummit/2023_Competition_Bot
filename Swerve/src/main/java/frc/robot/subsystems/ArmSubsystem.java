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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmSystemConstants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax armMotor;

  private DigitalInput upperLimitSwitch;
  private DigitalInput lowerLimitSwitch;

  private RelativeEncoder armEncoder;

  private SlewRateLimiter limiter;
  private double target;
  private boolean encoderCalibrated;

  public ArmSubsystem() {

    armMotor = new CANSparkMax(ArmSystemConstants.kArmMotorCANid, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armEncoder = armMotor.getEncoder();

    limiter = new SlewRateLimiter(0.75, -1.5, 0);
    target = 0.0;
    
  upperLimitSwitch = new DigitalInput(ArmSystemConstants.kUpperLimitSwitchPort);

  lowerLimitSwitch = new DigitalInput(ArmSystemConstants.kLowerLimitSwitchPort);
  encoderCalibrated = false;
  
}


  public boolean atUpperLimit(){
    return upperLimitSwitch.get(); 
  }
  public boolean atLowerLimit(){
    return lowerLimitSwitch.get(); 
  }

  public boolean isCalibrated(){
    return encoderCalibrated;
  }

  public void resetEncoder(){
    armEncoder.setPosition(0);
    encoderCalibrated = true;
  }

  public double getPosition(){
    return armEncoder.getPosition();
  }

  public void Up(){
    if(!atUpperLimit()){
      target = 1;
    }
  }
  public void Up(double speed){
    if(speed < 0){
      return; //should throw an error
    }
    if(!atUpperLimit()){
      target = speed;
    }
  }
  
  public void Down(){
    if(!atLowerLimit()){
      target = -0.75;
    }
  }

  public void Down(double speed){
    if(speed < 0){
      return;
    }
    if(!atLowerLimit()){
      target = 0-speed;
    }
  }
  public void stop(){
    target = 0.0;
  }

  public void hardStop(){
    armMotor.stopMotor();
    limiter.reset(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //prevent going the wrong direction when a limit switch is tripped.
    if(atUpperLimit() && armMotor.get() > 0){
      hardStop();
      return;
    }
    if(atLowerLimit() && armMotor.get() < 0){
      hardStop();
      return;
    }

    //apply slew limiter
    double limitedMove = limiter.calculate(target);


    //TODO: limit speeds as limit switches are approached.
    double lowerApproachSpeed = -0.2;
    double lowerSpeedControlZone = 30;
    if(getPosition() < lowerSpeedControlZone && limitedMove < 0){
      if(limitedMove < lowerApproachSpeed){
        limitedMove = lowerApproachSpeed;
      }
    }

    double upperApproachSpeed = 0.2;
    double upperSpeedControlZone = 305;
    if(getPosition() > upperSpeedControlZone && limitedMove > 0){
      if(limitedMove > upperApproachSpeed){
        limitedMove = upperApproachSpeed;
      }
    }

    //apply the limited motor movement.
    armMotor.set(limitedMove);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Power", () -> armMotor.get() , null);
      builder.addDoubleProperty("Position", () -> getPosition() , null);
      builder.addBooleanProperty("UpperLimit", () -> atUpperLimit() , null);
      builder.addBooleanProperty("LowerLimit", () -> atLowerLimit(), null);
  }

}
