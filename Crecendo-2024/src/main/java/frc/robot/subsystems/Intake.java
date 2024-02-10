
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public static TalonFX intakeRoller;



  public Intake() {

  }

  static{

  intakeRoller = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "can");
  }

  @Override
  public void periodic() {
    
  }

  //Sets the intake rollers
  public static void setRollers(double speed){
    intakeRoller.set(speed);
  }

  //Sets the lift to a certain speed
  

  //Gets lift position


  //Gets roller position


  public static StatusSignal<Double> getRollerCurrent(){
    return intakeRoller.getSupplyCurrent();
  }

  //Resets lift encoder


  //Resets roller encoder


  //Get value of the intake lift limit switch

}
