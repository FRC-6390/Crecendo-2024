package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.sensors.IRBBSensor;
 
public class Intake extends SubsystemBase {
 
  public static TalonFX centerIntakeRoller;
  public static TalonFX fullWidthIntakeRoller;
  public static TalonFX feedingRollers;
  public static IRBBSensor lowerIntakeBeamBreak;
  public static IRBBSensor upperIntakeBeamBreak;
 
 
  public Intake()
  {

  }
 
  static
  {
 
  centerIntakeRoller = new TalonFX(Constants.INTAKE.CENTER_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  fullWidthIntakeRoller = new TalonFX(Constants.INTAKE.FULL_WIDTH_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  lowerIntakeBeamBreak = new IRBBSensor(Constants.INTAKE.BEAM_BREAK);
  upperIntakeBeamBreak = new IRBBSensor(4);
  feedingRollers = new TalonFX(Constants.INTAKE.FEEDNG_ROLLER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  //intakeBeamBreak = new DigitalInput(0);
  }
 
  //Get value of the intake lift limit switch
  public static boolean getLowerBeamBreak()
  {
    //false for triggered, otherwise true
  return lowerIntakeBeamBreak.isBroken();
  }
 
   public static boolean getUpperBeamBreak(){
 
    return upperIntakeBeamBreak.isBroken();
   }
 
  //Sets the intake rollers
  public static void setRollers(double speed, int num, int place)
  {
    if(num == 1)
    {
    centerIntakeRoller.set(0);
    fullWidthIntakeRoller.set(0);
    feedingRollers.set(0);
    }
    else if(num == 2)
    {
    centerIntakeRoller.set(speed);
    fullWidthIntakeRoller.set(speed);
    feedingRollers.set(0);
    }

    if(place ==3){
    centerIntakeRoller.set(speed);
    feedingRollers.set(speed);
    fullWidthIntakeRoller.set(0);
    }
    else if(place ==4)
    {
    centerIntakeRoller.set(0);
    feedingRollers.set(0);
    fullWidthIntakeRoller.set(0);
    }
 
  }
 
  //Sets the lift to a certain speed
 
 
  //Gets lift position
 
 
  //Gets roller position
 
 
  public static StatusSignal<Double> getRollerCurrent()
  {
    return centerIntakeRoller.getSupplyCurrent();
  }
 
  //Resets lift encoder
 
 
  //Resets roller encoder
 
 
  //Get value of the intake lift limit switch
 
 
  @Override
  public void periodic()
  {
    //System.out.println(getLowerBeamBreak());
  }
}