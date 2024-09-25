package frc.robot.utilities.swerve;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;


public class SwerveModule {
    public TalonFX driveMotor;
    public TalonFX rotationMotor;

    private CANcoder encoder;
    private PID pid;
    public double encoderConversion;
    public double maxSpeedMpS;

    

    public record SwerveModuleConfig(
    int driveMotor, 
    boolean driveMotorReversed, 
    int rotationMotor, 
    boolean rotationMotorReversed, 
    int encoder, 
    double encoderOffset, 
    String canbus,
    PIDConfig rotationPID,
    double encoderConversion,
    double maxSpeedMpS
    ) {
        public SwerveModuleConfig(
            int driveMotor, 
            boolean driveMotorReversed, 
            int rotationMotor, 
            boolean rotationMotorReversed, 
            int encoder, 
            double encoderOffset,
            PIDConfig rotationPID,
            double encoderConversion,
            double maxSpeedMpS){
            this(
                driveMotor, 
                driveMotorReversed, 
                rotationMotor, 
                rotationMotorReversed,
                encoder, 
                encoderOffset, 
                "can", 
                rotationPID,
                encoderConversion,
                maxSpeedMpS
            );
        }
    }

    private double encoderOffset;
    private static int instances = 0;

   private StatusSignal<Double> drivePos,driveVel, encoderPos;

    public static PIDController rotationPidController = new PIDController(0.25, 0, 0);

    public SwerveModule(SwerveModuleConfig config, ShuffleboardTab tab){
        maxSpeedMpS = config.maxSpeedMpS();
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        if(config.canbus() != null){
           // System.out.println(config.canbus());
            driveMotor = new TalonFX(config.driveMotor(), config.canbus());
            rotationMotor = new TalonFX(config.rotationMotor(), config.canbus());
            encoder = new CANcoder(config.encoder(), config.canbus());
        }else{
            driveMotor = new TalonFX(config.driveMotor());
            rotationMotor = new TalonFX(config.rotationMotor());
            encoder = new CANcoder(config.encoder());
        }
        encoderOffset = config.encoderOffset();
        driveMotor.setInverted(config.driveMotorReversed());
        rotationMotor.setInverted(config.rotationMotorReversed());
        TalonFXConfiguration con2 =  new TalonFXConfiguration();
        CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
        curr.SupplyCurrentLimitEnable = true;
        curr.SupplyCurrentLimit = 50; //used to be 60
        con2.withCurrentLimits(curr);
        encoderConversion = config.encoderConversion();
        driveMotor.getConfigurator().apply(con2);
        encoderPos=encoder.getPosition();
        
        
        
        CANcoderConfiguration  con = new CANcoderConfiguration();
        con.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        con.MagnetSensor.MagnetOffset = encoderOffset;
        encoder.getConfigurator().apply(con);
        pid = new PID(config.rotationPID()).setMeasurement(() -> getRotationMotorPosition());
        instances++;
        drivePos=driveMotor.getRotorPosition();
        driveVel=driveMotor.getRotorVelocity();
        resetEncoders();
        lock();
    }

    public double getDriveMotorVelocity(){

        return driveVel.getValueAsDouble()  * encoderConversion;
        
    }
    
    public double getDriveMotorPosition(){

       return drivePos.getValueAsDouble()* encoderConversion;
    }

    public double getRotationMotorPosition(){
        return getEncoderRadians();
    }

    public double getAbsolutePosition(){
        return encoderPos.getValueAsDouble() * Math.PI/180d;
    }

    public double getEncoderOffset(){
        return encoderOffset;
    }

    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    //ASK MATHIAS FOR THIS
    public double getEncoderRadians(){
        return (encoderPos.getValueAsDouble()*360 * Math.PI/180d);
    }

    public void resetEncoders(){
        driveMotor.setPosition(0);
        rotationMotor.setPosition(encoderPos.getValueAsDouble());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getEncoderRadians()));
    }

    public SwerveModulePosition getPostion(){
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getEncoderRadians()));
    }

    public void setDriveMotor(double speed){
        driveMotor.set(speed);
    }

    public void setRotationMotor(double speed){
        rotationMotor.set(speed);
    }

    public void setDesiredState(SwerveModuleState state){
        refresh();
        
    

        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();

            return;
        }

      state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / maxSpeedMpS);

        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void setToAngle(double angle){
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(angle));

        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void lock(){
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void unlock(){
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        rotationMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void refresh(){
        drivePos.refresh();
        driveVel.refresh();
        encoderPos.refresh();
    }

}