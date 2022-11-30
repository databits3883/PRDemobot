// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final VictorSP m_leftMotor = new VictorSP(1);
  private final VictorSP m_rightMotor = new VictorSP(0);
  private final VictorSP m_launchMotor = new VictorSP(3);
  private final VictorSP m_stageMotor = new VictorSP(2);
  
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_overrideStick = new Joystick(1);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.setInverted(true);
    
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    

    if (m_stick.getTrigger()){
      m_launchMotor.set(0.25);
    }
    else{
      m_launchMotor.set(0);
    }

    if (m_stick.getRawButton(2)){
      m_stageMotor.set(1);
    }
    else{
      m_stageMotor.set(0);
    }

    if (m_overrideStick.getTrigger()){
      m_robotDrive.arcadeDrive(-m_overrideStick.getY(), -m_overrideStick.getX());
    }
    else{
      m_robotDrive.arcadeDrive(-m_stick.getY()*(1f-m_overrideStick.getThrottle() + 0.25f), -m_stick.getX());
    }
  }
}
