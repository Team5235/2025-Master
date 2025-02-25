// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeStop;
import frc.robot.util.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.controls.StrictFollower;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsytem extends SubsystemBase {

  private SparkMax m_intake;
  private SparkMaxConfig intakeConfig;
  private double maxPwr = 0.5;


  /** Creates a new IntakeSubsytem. */
  public IntakeSubsytem() {

    /*
 * Initialize the SPARK MAX and get its encoder adn closed loop controller
 * objects for later use
 */

 m_intake = new SparkMax(Constants.CANIDs.INTAKE_ID, MotorType.kBrushed);

 /*
  * Create a new SPARK MAX configuration object. This will store the
  * configuraton paramaters for the SPARK MAX that we will set below
  *  ***** MOVE THIS TO CONFIG SECTION ****
  */
 
 intakeConfig = new SparkMaxConfig();
     /*
     * Apply the configuration to the SPARK MAX.
     * 
     * kResetSafeParamaters is used to get the SPARK MAX to a knows state. This
     * is useful in case the SPARK MAX is replaced.
     * 
     * kPersistParamaters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that occur
     * mid-operation.
     */
     m_intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
     
  setDefaultCommand(new IntakeStop(this));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    maxPwr = .25;
  }
  public void IntakeIN() {
    m_intake.set(maxPwr);

  }

  public void IntakeOUT() {
    m_intake.set(maxPwr * -1);
 
  }

  public void IntakeStop() {

    m_intake.set(0);
  }

}
