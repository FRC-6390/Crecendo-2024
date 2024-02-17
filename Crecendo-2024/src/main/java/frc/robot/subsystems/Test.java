// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.controlloop.PID;

public class Test extends SubsystemBase {
  /** Creates a new Test. */
  private TalonFX ArmMotor;
  private PID PID;

  private StatusSignal<Double> rotorPos;

 

  public Test() {    
    ArmMotor = new TalonFX(Constants.TEST.ARM_MOTOR, "can");
    PID = new PID(Constants.TEST.PID_config);
    rotorPos = ArmMotor.getRotorPosition();
    PID.setMeasurement(()->rotorPos.getValueAsDouble());
  }

  public void setHome(){
    ArmMotor.setPosition(0);
  }

  public void stopAll(){
    ArmMotor.set(0);
  }

  public boolean checkThreshold(){
   return true;
  }
  public void setPosition(double pos){
  double convertedValue = pos*Constants.TEST.ARM_MAX; 
  double speed = PID.calculate(convertedValue);
  ArmMotor.set(speed);
  }

 public void setHalf(){
  setPosition(0.5);
 }

  @Override
  public void periodic() {
    rotorPos.refresh();
    // This method will be called once per scheduler run
    //System.out.println(ArmMotor.getRotorPosition());
  }
}
