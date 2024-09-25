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
import edu.wpi.first.math.geometry.Translation2d;
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
    public SwerveMotor driveMotorRecord;
    public SwerveMotor rotationMotorRecord;
    public double encoderGearRatio;
    public Translation2d module_location;

    //SWERVE MOTOR RECORD
    public record SwerveMotor(int motor, boolean motorReversed, double gearRatio, double maxSpeedMetersPerSecond, String canbus)
    {
        public SwerveMotor(int motor, boolean motorReversed, double gearRatio, double maxSpeedMetersPerSecond)
        {
            this(motor, motorReversed, gearRatio, maxSpeedMetersPerSecond, "can");
        }
    }

    public record SwerveModuleConfig(SwerveMotor driveMotor,SwerveMotor rotationMotor,int encoder, double encoderOffset,PIDController rotationPID, double encoderGearRatio, Translation2d module_location) 
    {

    }

    private double encoderOffset;

    private StatusSignal<Double> drivePos,driveVel, encoderPos;

    //CHANGING
    public static PIDController rotationPidController;

    public SwerveModule(SwerveModuleConfig config){
        //INITIALIZING RECORD
        driveMotorRecord = config.driveMotor();
        rotationMotorRecord = config.rotationMotor();

        //ROTATION PID ENABLING -180 TO 180
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        //MOTOR INITIALIZATION
        driveMotor = new TalonFX(driveMotorRecord.motor(), driveMotorRecord.canbus());
        rotationMotor = new TalonFX(rotationMotorRecord.motor(), rotationMotorRecord.canbus());
        encoder = new CANcoder(config.encoder(), config.rotationMotor().canbus());

        //ENCODER OFFSETS INITIALIZED HERE
        encoderOffset = config.encoderOffset();

        //INVERTING MOTORS
        driveMotor.setInverted(driveMotorRecord.motorReversed());
        rotationMotor.setInverted(rotationMotorRecord.motorReversed());

        //CURRENT LIMITING
        TalonFXConfiguration con2 =  new TalonFXConfiguration();
        CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
        curr.SupplyCurrentLimitEnable = true;
        curr.SupplyCurrentLimit = 50; //used to be 60
        con2.withCurrentLimits(curr);
        driveMotor.getConfigurator().apply(con2);

        //ENCODER POSITION
        encoderPos=encoder.getPosition();
        
        //ENCODER CONFIGURATION
        CANcoderConfiguration  con = new CANcoderConfiguration();
        con.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        con.MagnetSensor.MagnetOffset = encoderOffset;
        encoderGearRatio = config.encoderGearRatio();
        encoder.getConfigurator().apply(con);

        //MODULE ROTATION PID
        rotationPidController = config.rotationPID();

        //DRIVE MOTOR POSITION AND VELOCITY
        drivePos=driveMotor.getRotorPosition();
        driveVel=driveMotor.getRotorVelocity();

        //MODULE LOCATION
        module_location = config.module_location();

        //RESET ENCODERS AND BRAKE MODE MODULES
        resetEncoders();
        lock();
    }

    public double getDriveMotorVelocity(){

        return driveVel.getValueAsDouble()  * driveMotorRecord.gearRatio();
        
    }
    
    public double getDriveMotorPosition(){

       return drivePos.getValueAsDouble()* driveMotorRecord.gearRatio();
    }

    public double getRotationMotorPosition(){
        return getEncoderRadians() * encoderGearRatio;
    }

    public double getAbsolutePositionRadians(){
        return encoderPos.getValueAsDouble() * Math.PI/180d * encoderGearRatio;
    }

    public double getEncoderOffset(){
        return encoderOffset;
    }

    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

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
        driveMotor.set(state.speedMetersPerSecond / driveMotorRecord.maxSpeedMetersPerSecond());

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