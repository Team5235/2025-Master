package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ArmConstants;

@SuppressWarnings("unused")

public class SetArmHome extends Command {

    
    ArmSubsystem armSub;

    private double armPosition;
    
    public SetArmHome(ArmSubsystem armSub, double armPosition) {
        this.armSub = armSub;
        this.armPosition = armPosition;
        addRequirements(armSub);
        }

    @Override
    public void initialize() {
            // Adjust the arm position here
            armSub.setArmPose(ArmConstants.ARM_HOME_POSE);
    }

    @Override
    public boolean isFinished() {
        return true;
   }

} // end of class