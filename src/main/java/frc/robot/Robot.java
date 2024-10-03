// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  
  public AddressableLED robotLEDs;
  public AddressableLEDBuffer robotLEDBuffer = new AddressableLEDBuffer(60);
  public String currentLightSignal = "IDLE";

  private final VictorSP m_leftMotor = new VictorSP(1);
  private final VictorSP m_rightMotor = new VictorSP(0);
  private final VictorSP m_launchMotor = new VictorSP(3);
  private final VictorSP m_stageMotor = new VictorSP(2);
  private final CANSparkMax m_hoodMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final PIDController m_rotController = new PIDController(1, 0, 0);
  private final PIDController m_driveController = new PIDController(1, 0, 0);
 // private final Pigeon2 gyro = new Pigeon2(4);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_overrideStick = new Joystick(1);
  private final String Logitech_Jotstick = "Logitech Attack 3";
  private final String Thrustmaster_Joystick = "T.16000M";
  private boolean isThrustmaster_Joystick = false;
  private double launchMotorSpeed= 0.5;
  private SendableChooser launchSpeedChooser;
  public int m_raiseHoodButtonID = 4;
  public int m_lowerHoodButtonID = 5;
  private int launchMotorTimer = 0;
  //Cycles needed to spin up Launch motor to max speed
  private final int launchMotorTimeToLaunchSpeed = 150/* at 0.5 */;//215 at 0.3
  private int animationCounter = 0;
  //Reduce staging speed due to better stage motor
  private double m_stageSpeed = 0.75;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.    
    m_leftMotor.setInverted(true);
    m_rotController.setTolerance(0.25, 0.5);    

    robotLEDs = new AddressableLED(8);
    robotLEDs.setLength(robotLEDBuffer.getLength());
    //Determine if Logitech joysticks are in use or the Thrustmaster
    if (Logitech_Jotstick.equals(m_stick.getName()))
    {
      System.out.println("Logitech Joystick Detected");
      isThrustmaster_Joystick = false;
    }
    else if (Thrustmaster_Joystick.equals(m_stick.getName()))
    {
      System.out.println("Thrustmaster Joystick Detected");
      isThrustmaster_Joystick = true;
      //todo: update chute buttons
      m_raiseHoodButtonID = 3;
      m_lowerHoodButtonID = 4;
    }
    else
    {
      isThrustmaster_Joystick = false;
      System.out.println("Unknown Joystick Detected - ["+m_stick.getName()+"]");
    }

    launchSpeedChooser = new SendableChooser<Double>();
    Shuffleboard.getTab("Demo Menu").addDouble("Throttle", this::getThrottle);
    
    launchSpeedChooser.setDefaultOption("50% [Default]", 0.5);
    launchSpeedChooser.addOption("75%", 0.75);
    launchSpeedChooser.addOption("100%", 1.0);
    launchSpeedChooser.addOption("25%", 0.25);
    launchSpeedChooser.addOption("10%", 0.1);
    launchSpeedChooser.addOption("0% (Off)", 0.0);
    Shuffleboard.getTab("Demo Menu").add(launchSpeedChooser);
    Shuffleboard.selectTab("Demo Menu");
    
  }

  @Override
  public void robotPeriodic(){
    switch (currentLightSignal) {
      case "LAUNCHER":
        VisualizeLauncher(new Color8Bit(100, 0, 100));
        break;
    
      default:
          WaveColorWithTime(new Color8Bit(edu.wpi.first.wpilibj.util.Color.kGreen));

        break;
    }

  }

  @Override
  public void teleopPeriodic() {

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    launchMotorSpeed = Double.valueOf(launchSpeedChooser.getSelected().toString());

    //If the override is enabled with the trigger held down, take control
    if (m_overrideStick.getTrigger()){
      m_robotDrive.arcadeDrive(-m_overrideStick.getY(), -m_overrideStick.getX());

      //Launch motor button 3, Stage motor button 2 on override stick
      if (m_overrideStick.getRawButton(3)){
        SpinUpLaunchMotor();
      }
      else{
        currentLightSignal = "IDLE";
        m_launchMotor.set(0);
        //Decrease the launch Motor timer as the motor slows down
        if (launchMotorTimer > 0) launchMotorTimer--;
      }
      //Checks if launcher button is pressed and launch motor is up to speed before staging
      if ((m_overrideStick.getRawButton(3))  && (launchMotorTimer > launchMotorTimeToLaunchSpeed)){
        m_stageMotor.set(m_stageSpeed);
      }
      else{
        m_stageMotor.set(0);
      }

 
    }  //End override enabled
    else{
      m_robotDrive.arcadeDrive(-m_stick.getY()*(getThrottle()), -m_stick.getX() * getThrottle());

      if (m_stick.getTrigger()){
        SpinUpLaunchMotor();
      }
      else{
        m_launchMotor.set(0);
        //Decrease the launch Motor timer as the motor slows down
        if (launchMotorTimer > 0) launchMotorTimer--;
        currentLightSignal = "IDLE";
      }
      //Checks if launcher button is pressed and launch motor is up to speed 
      // before staging can be used
      if ((m_stick.getTrigger())&& (launchMotorTimer > launchMotorTimeToLaunchSpeed)){
        m_stageMotor.set(m_stageSpeed);
      }
      else{
        m_stageMotor.set(0);
      }
     
    } //End not being overridden

    //this looks cool
    if (m_overrideStick.getRawButtonPressed(m_raiseHoodButtonID)){
      SetHoodMotor(0.1);
    }
    else if(m_overrideStick.getRawButtonReleased(m_raiseHoodButtonID)){
      SetHoodMotor(0);
    }
    
    if (m_overrideStick.getRawButtonPressed(m_lowerHoodButtonID)){
      SetHoodMotor(-0.1);
    }
    else if(m_overrideStick.getRawButtonReleased(m_lowerHoodButtonID)){
      SetHoodMotor(0);
    }
  }

//duncan is not amazing according to 
  private void SpinUpLaunchMotor(){
  
      m_launchMotor.set(launchMotorSpeed);
      //Increase the launch motor timer as motor speeds up
      if (launchMotorTimer <= launchMotorTimeToLaunchSpeed) launchMotorTimer++;
      currentLightSignal = "LAUNCHER";
  }

  private double getThrottle()
  {
    double throttle = 0;
    if (isThrustmaster_Joystick) 
      throttle = (1f-m_overrideStick.getThrottle());
    else
    { 
        
      throttle = (1 - m_overrideStick.getZ()) / 2;
      
    }

    if(throttle != 0){
        throttle =  (0.5) + (throttle*0.5);
      }
    return throttle;
  }

  public void SetHoodMotor(double speed){
    m_hoodMotor.set(speed);
  }

  private void WaveColorWithTime(Color8Bit color) {
    animationCounter +=5;
    if(animationCounter>360){
      animationCounter = 0;
    }
    //double timerDeg = Units.degreesToRadians(timer);
    for (int i = 0; i < robotLEDBuffer.getLength(); i++) {
      
      double brightness  = ((Math.sin( Units.degreesToRadians( i*5 + animationCounter ) ) + 1) / 2) / 1;
      robotLEDBuffer.setRGB(i, (int)(brightness * color.red), (int)(brightness * color.green), (int)(brightness * color.blue));
      //armLEDBuffer.setRGB(i, 100, 100, 100);
      
   }

    robotLEDs.setData(robotLEDBuffer);
    robotLEDs.start();
   
  }

  private void VisualizeLauncher(Color8Bit color) {
    
    //double timerDeg = Units.degreesToRadians(timer);
    for (int i = 0; i < robotLEDBuffer.getLength()/2; i++) {

      if( ((double)i / robotLEDBuffer.getLength()/2) < (double)launchMotorTimer/(launchMotorTimeToLaunchSpeed * 4)){
      //robotLEDBuffer.setRGB(i, color.red, color.green, color.blue);
        robotLEDBuffer.setLED(i, color);
        robotLEDBuffer.setLED(robotLEDBuffer.getLength() - (i + 1 ), color);
      }
      else{
        robotLEDBuffer.setRGB(i, 0, 0, 0);
        robotLEDBuffer.setRGB(robotLEDBuffer.getLength() - (i + 1 ), 0, 0, 0);
      }
     }

    robotLEDs.setData(robotLEDBuffer);
    robotLEDs.start();
   
  }
}
