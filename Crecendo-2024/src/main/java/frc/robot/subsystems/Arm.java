// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.sensors.Button;

public class Arm extends SubsystemBase {
  /** Creates a new Test. */
  private TalonFX ArmMotorLeft;
  private TalonFX ArmMotorRight;
  private PID PID;
  public double setpoint = 0;
  public double convertedValue = 0;
  public Button coastButton;
  private DebouncedJoystick joystick;
 // public DigitalInput limitSwitch = new DigitalInput(2);
  

  private StatusSignal<Double> rotorPos;
  public double maxPos = Constants.ARM.ARM_MAX;
  public StatusSignal<Double> amperage;
  public boolean shouldCoast;
 

  public Arm(DebouncedJoystick joystick) {    
    ArmMotorLeft = new TalonFX(Constants.ARM.ARM_MOTOR_LEFT, Constants.DRIVETRAIN.CANBUS);
    ArmMotorRight = new TalonFX(Constants.ARM.ARM_MOTOR_RIGHT, Constants.DRIVETRAIN.CANBUS);
    coastButton = new Button(new DigitalInput(1));
    this.joystick = joystick;
    joystick.one.onFalse(new InstantCommand(this::stopAll));
    // coastButton.whileFalse(new InstantCommand(this::motorCoast));    
    // coastButton.whileTrue(new InstantCommand(this::motorBrake));    

    PID = new PID(Constants.ARM.PID_config);
    rotorPos = ArmMotorLeft.getRotorPosition();
    amperage = ArmMotorLeft.getTorqueCurrent();
    PID.setMeasurement(()->rotorPos.getValueAsDouble());
    ArmMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    ArmMotorRight.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration con = new TalonFXConfiguration();
    CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
    curr.SupplyCurrentLimitEnable = true;
    curr.SupplyCurrentLimit = 80;
    con.withCurrentLimits(curr);
    motorBrake();
    //ArmMotorLeft.getConfigurator().apply(con);
    //ArmMotorRight.getConfigurator().apply(con);
  }

  public void setSpeed(double speed)
  {
    ArmMotorLeft.set(-speed);
    ArmMotorRight.set(speed);
  }

  public void setHome(){
    ArmMotorLeft.setPosition(0);
  }

  public void stopAll(){
    ArmMotorLeft.set(0);
    ArmMotorRight.set(0);
  }

  public boolean checkThreshold(){
   return true;
  }
  public void setPosition(double pos)
  {
  pos = Math.min(Math.max(pos, -1), 0);
  setpoint = pos;
  convertedValue = (maxPos)*setpoint;
  double speed = PID.calculate(convertedValue);
  setSpeed(-speed);
  }

  public boolean atPosition()
  {
    return Math.abs((maxPos * setpoint) - rotorPos.getValueAsDouble()) < 0.6; 
    //return  rotorPos.getValueAsDouble() >= ((maxPos * setpoint)- 0.2) && rotorPos.getValueAsDouble() <= ((maxPos * setpoint) - 0.2); 
  }

public boolean override(){
  return joystick.one.getAsBoolean();
}
 public void setHalf(){
  setPosition(0.5);
 }

 public void motorBrake(){
  ArmMotorLeft.setNeutralMode(NeutralModeValue.Brake);
  ArmMotorRight.setNeutralMode(NeutralModeValue.Brake);
}

public void motorCoast(){
  ArmMotorLeft.setNeutralMode(NeutralModeValue.Coast);
  ArmMotorRight.setNeutralMode(NeutralModeValue.Coast);
  System.out.println("///////////////////////////////Couast////////////////////////////");
}
    // }
  @Override
  public void periodic() {
    if (joystick.one.getAsBoolean()){
      double input = joystick.leftY.getAsDouble();
      double newRange = (0 - (-1));
      double oldRange = (1-(-1));
      double joystickPos = (((input - (-1)) * newRange)/oldRange)+(-1);
      setPosition(joystickPos);
    }
    rotorPos.refresh();

    amperage.refresh();

    // if(limitSwitch.get()){
    //   setHome();
    // }
    // else{
    //   setSpeed(0.2);
    // }
    // SmartDashboard.putString("Rotor Pos", Double.toString(rotorPos.getValueAsDouble()));
    // SmartDashboard.putString("Max Pos", Double.toString(maxPos));
    // SmartDashboard.putString("Converted Value", Double.toString(maxPos * setpoint));
    // SmartDashboard.putString("Calculated Speed", Double.toString(PID.calculate(maxPos * setpoint)));

    // SmartDashboard.putBoolean("Difference", atPosition());

    
    // convertedValue = (maxPos)*setpoint;
    // double speed = PID.calculate(convertedValue);
    // setSpeed(speed * -1);
if (DriverStation.isDisabled()){
  if (coastButton.isPressed()){
    motorCoast();
  }
else{
motorBrake();
}
}

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
