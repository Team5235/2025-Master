
package frc.robot.util;

//import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FollowArmConfigs {

    public static void applyArmConfigs(TalonFX m_arm) {

        // Set the default configuration for the arm motor
        m_arm.getConfigurator().apply(new TalonFXConfiguration());

        //configure gear ratio and dissable continuous wrap
        TalonFXConfiguration armwrap0 = new TalonFXConfiguration();
           // armwrap0.Feedback.SensorToMechanismRatio = 1; // adjust to whatever needed based on gearing
           // armwrap0.Feedback.RotorToSensorRatio = 1;     // adjust to whatever needed based on gearing
            armwrap0.ClosedLoopGeneral.ContinuousWrap = false; //Dissable Continuous wrap - impportant for absolute positioning
        m_arm.getConfigurator().apply(armwrap0,0.050);

       // not needed done in config below//
       // m_arm.setInverted(true); // Set to true if you want to invert the motor direction

        /* Gains or configuration of arm motor for config slot 0 */
        var armGains0 = new Slot0Configs();
        armGains0.GravityType = GravityTypeValue.Arm_Cosine; /* .Elevator_Static | .Arm_Cosine */
        armGains0.kP = 4.50; /* Proportional Gain */        //4.80   // was 0.50
        armGains0.kI = 0.00; /* Integral Gain */
        armGains0.kD = 0.10; //0.01; /* Derivative Gain */             // was zero
        armGains0.kV = 0.12; /* Velocity Feed Forward Gain */  // was zero
        armGains0.kS = 0.25; //0.25; /* Static Feed Forward Gain */    // was zero
        armGains0.kA = 0.00; /* Acceleration Feedforward */
        armGains0.kG = 0.25; /* Gravity Feedfoward */

        // set Motion Magic settings
        var armMotionMagic0 = new MotionMagicConfigs();
        armMotionMagic0.MotionMagicCruiseVelocity = ArmConstants.ARM_MAX_VEL; 
        armMotionMagic0.MotionMagicAcceleration = ArmConstants.ARM_MAX_ACCEL; 
        armMotionMagic0.MotionMagicJerk = ArmConstants.ARM_JERK;

        var armSoftLimit0 = new SoftwareLimitSwitchConfigs();
        armSoftLimit0.ForwardSoftLimitEnable = true;
        armSoftLimit0.ForwardSoftLimitThreshold = ArmConstants.ARM_FWD_LIMIT;
        armSoftLimit0.ReverseSoftLimitEnable = true;
        armSoftLimit0.ReverseSoftLimitThreshold = ArmConstants.ARM_REV_LIMIT;

        var armCurrent0 = new CurrentLimitsConfigs();
        armCurrent0.StatorCurrentLimitEnable = true;
        armCurrent0.StatorCurrentLimit = ArmConstants.ARM_STATOR_LIMIT;
        armCurrent0.SupplyCurrentLimitEnable = true;
        armCurrent0.SupplyCurrentLimit = ArmConstants.ARM_SUPPLY_LIMIT;

        //set motor to inverted
        var m_invert = new MotorOutputConfigs();
           // set invert to CW+ and apply config change
		   //m_invert.Inverted = InvertedValue.Clockwise_Positive;
        //m_invert.Inverted = InvertedValue.CounterClockwise_Positive;
        // set opposite main motor for follower
        m_invert.Inverted = InvertedValue.CounterClockwise_Positive;
        m_invert.NeutralMode = NeutralModeValue.Brake;
  
        /* Apply Configs */
        m_arm.getConfigurator().apply(armGains0, 0.050);
        m_arm.getConfigurator().apply(armMotionMagic0, 0.050);
        m_arm.getConfigurator().apply(armSoftLimit0, 0.050);
        m_arm.getConfigurator().apply(armCurrent0, 0.050);
        //m_arm.getConfigurator().apply(armwrap0,0.050);
        m_arm.getConfigurator().apply(m_invert, 0.050);

  
    }        
} // end of class ArmIO
