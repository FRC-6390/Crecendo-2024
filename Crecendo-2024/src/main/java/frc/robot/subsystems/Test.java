// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.utilities.controlloop.PID;
// // import io.github.oblarg.oblog.Logger;
// // import io.github.oblarg.oblog.annotations.Config;

// public class Test extends SubsystemBase {
//   /** Creates a new Test. */
//   private TalonFX ArmMotor;
//   private PID PID;
//   public double setpoint = 1;
//   public double convertedValue = 0;
//   public DigitalInput limitSwitch = new DigitalInput(0);
  

//   private StatusSignal<Double> rotorPos;
//   public double maxPos = 0;
//   public StatusSignal<Double> amperage;
//   public boolean shouldCoast;
//   private boolean homePosSet;
 

//   public Test() {    
//     ArmMotor = new TalonFX(Constants.TEST.ARM_MOTOR, "can");
//     PID = new PID(Constants.TEST.PID_config);
//     rotorPos = ArmMotor.getRotorPosition();
//     amperage = ArmMotor.getTorqueCurrent();
//     ArmMotor.getPosition();
//     PID.setMeasurement(()->rotorPos.getValueAsDouble());
//     ArmMotor.setNeutralMode(NeutralModeValue.Brake);
//   }

//   public void setHome(){
//     ArmMotor.setPosition(0);
//   }

//   public boolean atPosition()
//   {
//     return PID.calculate(convertedValue) <= 0.2 && PID.calculate(convertedValue) >= -0.2;
//   }

//   public void stopAll(){
//     ArmMotor.set(0);
//   }

//   public boolean checkThreshold(){
//    return true;
//   }
//   public void setPosition(double pos)
//   {
//   setpoint = pos;
//   }

//  public void setHalf(){
//   setPosition(0.5);
//  }

//     // @Config.ToggleButton
//     // void enableCoast(boolean shouldCoast){
//     //   this.shouldCoast = shouldCoast;
      
//     // }
//   @Override
//   public void periodic() {
//     if (shouldCoast == true){
//       ArmMotor.setNeutralMode(NeutralModeValue.Coast);

//     }
//     else{
//       ArmMotor.setNeutralMode(NeutralModeValue.Brake);
//     }
//     rotorPos.refresh();
//     amperage.refresh();
//     // This method will be called once per scheduler run

//     System.out.println(maxPos);
//     convertedValue = (maxPos)*setpoint;
//     double speed = PID.calculate(convertedValue);
//     if(!homePosSet){
//       ArmMotor.set(0.1);
//       if(!limitSwitch.get()){
//         setHome();
//         maxPos = -7;
//         homePosSet = true;
//       }
//     }else{
//       ArmMotor.set(speed);

//     }


    
    
//   }
// }
