// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//importations
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  // Declaring Motors
  private CANSparkMax frontRightMotor;
  private CANSparkMax backRightMotor;
  private CANSparkMax frontLeftMotor;
  private CANSparkMax backLeftMotor;

  // Declaring encoders
  private CANEncoder frontRightEncoder;
  private CANEncoder backRightEncoder;
  private CANEncoder frontLeftEncoder;
  private CANEncoder backLeftEncoder;

  // Declaring Mecanum Drive
  private MecanumDrive mecanumDrive;

  // Declares NavX
  private AHRS NavX;

  // declares Field Orientation Boolean
  boolean fieldOriented;

  // used for slow mode and turbo mode
  private double speedScalar;
  

  

  

  public DriveSubsystem() 
  {
    //Instantiates Drive Motors
    frontRightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR);
    backRightMotor = new CANSparkMax(BACK_RIGHT_MOTOR);
    frontLeftMotor= new CANSparkMax(FRONT_LEFT_MOTOR);
    backLeftMotor = new CANSparkMax(BACK_LEFT_MOTOR);

    //Instantiates Mecanum Drive
    mecanumDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  

    //Sets current Limit
    frontRightMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    backRightMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    frontLeftMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    backLeftMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    //Sets Voltage Compensation
    frontRightMotor.enableVoltageCompensation(VOLTAGE_COMP);
    backRightMotor.enableVoltageCompensation(VOLTAGE_COMP);
    frontLeftMotor.enableVoltageCompensation(VOLTAGE_COMP);
    backLeftMotor.enableVoltageCompensation(VOLTAGE_COMP);
    
    //Sets Ramp Rate
    frontRightMotor.setOpenLoopRampRate(RAMP_RATE);
    backRightMotor.setOpenLoopRampRate(RAMP_RATE);
    frontLeftMotor.setOpenLoopRampRate(RAMP_RATE);
    backLeftMotor.setOpenLoopRampRate(RAMP_RATE);

    //Sets PID Values for Drive Motors
    setPID(frontRightMotor);
    setPID(backRightMotor);
    setPID(frontLeftMotor);
    setPID(backLeftMotor);

   




    
    
  }
  
  //configures PID values for the motor in the constructor
  public void setPID(CANSparkMax motor) {
    CANPIDController pid = motor.getPIDController();
    pid.setP(KP);
    pid.setI(KI);
    pid.setD(KD);
    pid.setIZone(KIZ);
    pid.setFF(KFF);
    pid.setOutputRange(K_MIN_OUTPUT, K_MAX_OUTPUT);
    motor.setSmartCurrentLimit(CURRENT_LIMIT);
    motor.enableVoltageCompensation(VOLTAGE_COMP);
  }

  //Method that allows the robot to drive
  public void drive(double xSpeed, double ySpeed, double zSpeed)
  {
   if (fieldOriented)
   {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zSpeed, NavX.getAngle());
   }
   else
   {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zSpeed);
   }
  }

  //Method that allows for Field Oriented driving to be toggled
  public void toggleFieldOriented(){
    fieldOriented = !fieldOriented;
   }

   //sets the speedScalar
   public void setSpeedScalar(double scalar){
    speedScalar = scalar;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is field oriented", fieldOriented);
  }
}
