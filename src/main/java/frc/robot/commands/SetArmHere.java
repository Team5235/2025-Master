package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.fasterxml.jackson.databind.deser.SettableBeanProperty.Delegating;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.util.ArmConstants;
import edu.wpi.first.units.measure.Angle;

@SuppressWarnings("unused")

public class SetArmHere extends Command {

    
    LiftArmSubsystem liftArmSub;

    private Angle armPosition;
    
    public SetArmHere(LiftArmSubsystem liftArmSub, Angle armPosition) {
        this.liftArmSub = liftArmSub;
        this.armPosition = armPosition;
        addRequirements(liftArmSub);
        }

    @Override
    public void initialize() {
            // Adjust the arm position here
            liftArmSub.HoldArmPos( liftArmSub.getArmPosAngle());
    }

    @Override
    public boolean isFinished() {
        return true;
   }

} // end of class