// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.ArmConfigs;
import frc.robot.util.Constants;
import frc.robot.util.FollowArmConfigs;
import frc.robot.commands.ArmStop;

public class LiftArmSubsystem extends SubsystemBase {
      // Declare a variable for the motor to be controlled
      private final TalonFX m_arm;
      private final TalonFX m_follow_arm;
      private double current_speed;

      // class member variable for an instance of the MotionMagicVoltage class
      final MotionMagicVoltage m_armPose = new MotionMagicVoltage(0); // Initializes to position 0

  /** Creates a new LiftArmSubsystem. */
  public LiftArmSubsystem() {

       // Initialize the motor in the constructor with the motor ID and optional canbus
       // ID
        m_arm = new TalonFX(Constants.CANIDs.ARM_ID);
        m_follow_arm = new TalonFX(Constants.CANIDs.FOLLOW_ARM_ID);

        // Apply configurations from the ArmConfigs file to the motor
        ArmConfigs.applyArmConfigs(m_arm);
        FollowArmConfigs.applyArmConfigs(m_follow_arm);

      setDefaultCommand(new ArmStop(this));


  }/* End of the class-method */



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //m_arm.set(current_speed);
    current_speed = .1;

    SmartDashboard.putNumber("ArmSpeed", m_arm.get());
    SmartDashboard.putNumber("FollowArmSpeed", m_follow_arm.get());
  }

     // public StatusSignal<Double> getArmPos() {
    public StatusSignal<Angle> getArmPos() {
        /* Reusing from drivetrain subsystem */
        return m_arm.getPosition();
        
    }

    public Angle getArmPosAngle() {
      /* Reusing from drivetrain subsystem */

      return m_armPose.getPositionMeasure();
      
  }

    public void HoldArmPos(Angle armPose) {
      //double adjarmpose = armPose/(2 * Math.PI);
      //m_arm.setControl(m_armPose.withPosition(armPose));
      m_arm.setControl(m_armPose.withPosition(armPose));

      // showArmTelemetry();
          
  }
    
  public void ArmLift() {
    m_arm.set(current_speed);
    //m_arm.setControl( m_armPose.withFeedForward(current_speed ));
    m_follow_arm.setControl(new StrictFollower(m_arm.getDeviceID()));
  }

  public void ArmLower() {
    m_arm.set(current_speed * -1);
    //m_arm.setControl( m_armPose.withFeedForward(current_speed * -1));
    m_follow_arm.setControl(new StrictFollower(m_arm.getDeviceID()));
  }

  public void ArmStop() {

    m_arm.set(0);
    m_follow_arm.setControl(new StrictFollower(m_arm.getDeviceID()));
  }



}
