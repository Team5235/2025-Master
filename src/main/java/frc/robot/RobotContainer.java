// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LiftArmSubsystem;
//added imports
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.ArmLift;
import frc.robot.commands.ArmLower;
import frc.robot.commands.ArmStop;
import frc.robot.commands.IntakeIN;
import frc.robot.commands.IntakeOUT;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbStop;
//import frc.robot.commands.SetArmHome;
//import frc.robot.commands.SetArmPosition;

import frc.robot.util.Constants;
import frc.robot.util.ArmConstants;

public class RobotContainer {
    //private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final CommandXboxController m_operatorController = new CommandXboxController(1); // My joystick
    //private final CommandXboxController[] Controllers = new CommandXboxController[] {  m_operatorController };

     public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // added subsystems
    ArmSubsystem arm = new ArmSubsystem();
    LiftArmSubsystem lift_Arm = new LiftArmSubsystem();
    IntakeSubsytem intake_2025 = new IntakeSubsytem();
    ClimbSubsystem climber = new ClimbSubsystem();

    public RobotContainer() {

       lift_Arm.setDefaultCommand(new ArmStop(lift_Arm));
       intake_2025.setDefaultCommand(new IntakeStop(intake_2025));
       climber.setDefaultCommand(new ClimbStop(climber));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        m_operatorController.y().whileTrue(new ArmLift(lift_Arm));
        m_operatorController.x().whileTrue(new ArmLower(lift_Arm));

        m_operatorController.a().whileTrue(new InstantCommand(() -> arm.setArmPose(ArmConstants.ARM_MID_POSE)));
        //m_operatorController.b().whileTrue(new InstantCommand(() -> lift_Arm.HoldArmPos(lift_Arm.getArmPosAngle())));

        /* old version which will move to a set position
            // m_operatorController.y().whileTrue(new InstantCommand(() -> arm.setArmPose(ArmConstants.ARM_HOME_POSE)));
            // m_operatorController.b().whileTrue(new InstantCommand(() -> arm.setArmPose(ArmConstants.ARM_MID_POSE)));
        */
        m_operatorController.leftBumper().whileTrue(new IntakeIN(intake_2025));
        m_operatorController.rightBumper().whileTrue(new IntakeOUT(intake_2025));

        // activate the climber ??? do we need a timer/counter on this ???
        m_operatorController.povUp().whileTrue(new Climb(climber));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
