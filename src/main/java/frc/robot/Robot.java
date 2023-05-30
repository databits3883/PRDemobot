// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import javax.lang.model.util.ElementScanner6;

//import com.ctre.phoenix.sensors.Pigeon2;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final VictorSP m_leftMotor = new VictorSP(1);
  private final VictorSP m_rightMotor = new VictorSP(0);
  private final VictorSP m_launchMotor = new VictorSP(3);
  private final VictorSP m_stageMotor = new VictorSP(2);
  private final PIDController m_rotController = new PIDController(1, 0, 0);
  private final PIDController m_driveController = new PIDController(1, 0, 0);
 // private final Pigeon2 gyro = new Pigeon2(4);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_overrideStick = new Joystick(1);
  private final String Logitech_Jotstick = "Logitech Attack 3";
  private final String Thrustmaster_Joystick = "T.16000M";
  private boolean isThrustmaster_Joystick = false;
  private final double launchMotorSpeed= 0.3;
  private int launchMotorTimer = 0;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
    m_leftMotor.setInverted(true);
    m_rotController.setTolerance(0.25, 0.5);    
    if (Logitech_Jotstick.equals(m_stick.getName()))
    {
      System.out.println("Logitech Joystick Detected");
    }
    else if (Thrustmaster_Joystick.equals(m_stick.getName()))
    {
      System.out.println("Thrustmaster Joystick Detected");
      isThrustmaster_Joystick = true;
    }
    else
    {
      System.out.println("Unknown Joystick Detected - ["+m_stick.getName()+"]");
    }

    
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    /*
    Vector2d input = new Vector2d(m_stick.getX(), m_stick.getY());
    input.rotate(gyro.getYaw());

    if (m_rotController.atSetpoint()){
      m_robotDrive.arcadeDrive(m_driveController.calculate(input.magnitude()), 0f);
    }
    else{
      m_robotDrive.arcadeDrive(0f, m_rotController.calculate(Math.toDegrees(Math.atan2(input.y, input.x))));
    }*/
    
    

    if (m_overrideStick.getTrigger()){
      m_robotDrive.arcadeDrive(-m_overrideStick.getY(), -m_overrideStick.getX());

      //Launch motor button 3, Stage motor button 2 
      if (m_overrideStick.getRawButton(3)){
        m_launchMotor.set(launchMotorSpeed);
        if (launchMotorTimer <= 215) launchMotorTimer++;
      }
      else{
        m_launchMotor.set(0);
        if (launchMotorTimer > 0) launchMotorTimer--;
      }
      //Checks if launcher button is pressed and launch motor is up to speed before staging
      if ((m_overrideStick.getRawButton(3)) && (m_overrideStick.getRawButton(2)) && (launchMotorTimer > 215)){
        m_stageMotor.set(1);
      }
      else{
        m_stageMotor.set(0);
      }

    }
    else{
      m_robotDrive.arcadeDrive(-m_stick.getY()*(getThrottle()), -m_stick.getX());

      if (m_stick.getTrigger()){
        m_launchMotor.set(launchMotorSpeed);
        if (launchMotorTimer <= 215) launchMotorTimer++;
      }
      else{
        m_launchMotor.set(0);
        if (launchMotorTimer > 0) launchMotorTimer--;
      }
      //Checks if launcher button is pressed and launch motor is up to speed before staging
      if ((m_stick.getTrigger()) && (m_stick.getRawButton(2)) && (launchMotorTimer > 215)){
        m_stageMotor.set(1);
      }
      else{
        m_stageMotor.set(0);
      }
  
     
    }


  }

  private double getThrottle()
  {
    double throttle = 0;
    if (isThrustmaster_Joystick) 
      throttle = (1f-m_overrideStick.getThrottle() + 0.25f);
    else
    { 
      double z_throttle = m_overrideStick.getZ();
      double z_atan = Math.atan(z_throttle);
      double z_cubed = Math.pow(z_throttle, 3);
      double z_cubeRoot = Math.pow(z_throttle, (double) 0.33);
      double z_squared = Math.pow(z_throttle, 2);
      double z_squareRoot = Math.pow(z_throttle, (double) 0.5);
      //Logitech Z is -1 to 1, convert to 0-1.
      double z_value = (((z_cubed)/* convert to 0-2 */ ) / 2 /*concert to 0-1 */);
     
      //co
      //z_value = z_value * .75; 
      //
      //z_value = z_value + .25;
  
     throttle = (1f-z_value);
     
      
    }
    return throttle;
  }
}
