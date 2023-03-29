package frc.robot.subsystems;

import javax.management.modelmbean.ModelMBeanConstructorInfo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmSystemConstants;
import edu.wpi.first.wpilibj.DigitalInput;

/** A robot arm subsystem that moves with a motion profile. */
public class PIDArmSubsystem extends ProfiledPIDSubsystem {
    private CANSparkMax armMotor;
    private RelativeEncoder armEncoder;

    private double currentOutput;

    private double maxPosition = 343;

    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    private boolean encoderCalibrated;
    //private final PWMSparkMax m_motor = new PWMSparkMax(ArmSystemConstants.kMotorPort);
  
    //private final Encoder m_encoder =
    //  new Encoder(ArmSystemConstants.kEncoderPorts[0], ArmSystemConstants.kEncoderPorts[1]);
  /*private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmSystemConstants.kSVolts, ArmSystemConstants.kGVolts,
          ArmSystemConstants.kVVoltSecondPerRad, ArmSystemConstants.kAVoltSecondSquaredPerRad);
*/
  /** Create a new ArmSubsystem. */
  public PIDArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmSystemConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmSystemConstants.kMaxVelocityUnitsPerSecond,
                ArmSystemConstants.kMaxAccelerationUnitsPerSecSquared)),
        0);
    //m_encoder.setDistancePerPulse(ArmSystemConstants.kEncoderDistancePerPulse);
    armMotor = new CANSparkMax(ArmSystemConstants.kArmMotorCANid, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armEncoder = armMotor.getEncoder();

    upperLimitSwitch = new DigitalInput(ArmSystemConstants.kUpperLimitSwitchPort);

    lowerLimitSwitch = new DigitalInput(ArmSystemConstants.kLowerLimitSwitchPort);
    encoderCalibrated = false;

    // Start arm at rest in neutral position
    //setGoal(ArmSystemConstants.kArmOffsetUnits);
    enable(); //only enable when configured
    //System.out.println("PID Arm Init");
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
      setGoal(maxPosition);
    }
  }
  
  public void Down(){
    if(!atLowerLimit()){
      setGoal(10);
    }
  }

  public void stop(){
    this.
    setGoal(getPosition());
  }

  public void hardStop(){
    armMotor.stopMotor();
  }

  public void RawUp(double speed){
    if(speed < 0){
      return; //should throw an error
    }
    if(!atUpperLimit()){
      armMotor.set(speed);
    }
  }

  public void RawDown(double speed){
    if(speed < 0){
      return;
    }
    if(!atLowerLimit()){
      armMotor.set(0-speed);
    }
  }


  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    //double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output

    /*if(output < 0.05){
      output = 0;
    }*/

    if(output > 0 && atUpperLimit()){
        armMotor.set(0);
    } else if(output < 0 && atLowerLimit()){
        armMotor.set(0);
        // handle uncalibrated
    } else {
        armMotor.set(output); // + feedforward);
        currentOutput = output;
    }
  }

  @Override
  public double getMeasurement() {
    return getPosition();
  }


    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Power", () -> armMotor.get() , null);
        builder.addDoubleProperty("Position", () -> getPosition() , null);
        builder.addBooleanProperty("UpperLimit", () -> atUpperLimit() , null);
        builder.addBooleanProperty("LowerLimit", () -> atLowerLimit(), null);
        builder.addDoubleProperty("Output", () -> currentOutput, null);
    }
}
