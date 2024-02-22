// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.controlloop.PID;

public class Arm extends SubsystemBase {
  /** Creates a new Test. */
  private TalonFX ArmMotorLeft;
  private TalonFX ArmMotorRight;
  private PID PID;
  public double setpoint = 0;
  public double convertedValue = 0;
 // public DigitalInput limitSwitch = new DigitalInput(2);
  

  private StatusSignal<Double> rotorPos;
  public double maxPos = Constants.ARM.ARM_MAX;
  public StatusSignal<Double> amperage;
  public boolean shouldCoast;
  private boolean homePosSet;
 

  public Arm() {    
    ArmMotorLeft = new TalonFX(Constants.ARM.ARM_MOTOR_LEFT, Constants.DRIVETRAIN.CANBUS);
    ArmMotorRight = new TalonFX(Constants.ARM.ARM_MOTOR_RIGHT, Constants.DRIVETRAIN.CANBUS);
    PID = new PID(Constants.ARM.PID_config);
    rotorPos = ArmMotorLeft.getRotorPosition();
    amperage = ArmMotorLeft.getTorqueCurrent();
    PID.setMeasurement(()->rotorPos.getValueAsDouble());
    ArmMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    ArmMotorRight.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setSpeed(double speed)
  {
    ArmMotorLeft.set(-speed);
    ArmMotorRight.set(speed);
  }

  public void setHome(){
    ArmMotorLeft.setPosition(0);
  }

  public boolean atPosition()
  {
    return PID.calculate(convertedValue) <= 0.2 && PID.calculate(convertedValue) >= -0.2;
  }

  public void stopAll(){
    ArmMotorLeft.set(0);
  }

  public boolean checkThreshold(){
   return true;
  }
  public void setPosition(double pos)
  {
  setpoint = pos;
  }

 public void setHalf(){
  setPosition(0.5);
 }

    // }
  @Override
  public void periodic() {
    rotorPos.refresh();
    amperage.refresh();
    System.out.println("Rotor Pos" + Double.toString(rotorPos.getValueAsDouble()));
    System.out.println("Max Pos" + Double.toString(maxPos));
    System.out.println("Converted Value" + Double.toString(maxPos * setpoint));
    System.out.println("Calculated Speed" + Double.toString(PID.calculate(maxPos * setpoint)));
    
    SmartDashboard.putString("Rotor Pos", Double.toString(rotorPos.getValueAsDouble()));
    SmartDashboard.putString("Max Pos", Double.toString(maxPos));
    SmartDashboard.putString("Converted Value", Double.toString(maxPos * setpoint));
    SmartDashboard.putString("Calculated Speed", Double.toString(PID.calculate(maxPos * setpoint)));
    
    convertedValue = (maxPos)*setpoint;
  double speed = PID.calculate(convertedValue);
  setSpeed(speed * -1);

    // if (shouldCoast == true){
    //   ArmMotor.setNeutralMode(NeutralModeValue.Coast);

    // }
    // else{
    //   ArmMotor.setNeutralMode(NeutralModeValue.Brake);
    // }
    // rotorPos.refresh();
    // amperage.refresh();
    // // This method will be called once per scheduler run

    // System.out.println(maxPos);
    // convertedValue = (maxPos)*setpoint;
    // double speed = PID.calculate(convertedValue);
    // if(!homePosSet){
    //   setSpeed(0.1);
    //   if(!limitSwitch.get()){
    //     setHome();
    //     maxPos = -7;
    //     homePosSet = true;
    //   }
    // }else{
   // setSpeed(speed);

   // }


    
    
  }
}
