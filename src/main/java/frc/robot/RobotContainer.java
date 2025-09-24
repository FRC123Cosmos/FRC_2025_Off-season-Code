// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.PulseScorerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ScorerSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  // private final WinchSubsystem winch = new WinchSubsystem();
  private final ScorerSubsystem scorer = new ScorerSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverControllerCommand =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController coPilotControllerCommand =
      new CommandXboxController(OIConstants.kCoPilotControllerPort);
  private final CommandGenericHID coPilotSecondControllerCommand =
      new CommandGenericHID(OIConstants.kCoPilotSecondControllerPort);

  private final XboxController copilotController = new XboxController(OIConstants.kCoPilotControllerPort);

  private final SendableChooser<Command> autoChooser;

  private final SendableChooser<AutoPos> autoPosition;

  private final Pose2d leftAlign = new Pose2d(-0.17, 0.0, Rotation2d.fromDegrees(0));
  private final Pose2d rightAlign = new Pose2d(0.17, 0.0, Rotation2d.fromDegrees(0));

  private double autoDelay;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CommandScheduler.getInstance().registerSubsystem(vision);
    /*---------------------------------------------Only For Center Auton----------------------------------------------- */

    new EventTrigger("elevatorUpL1").onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1), elevator));

    new EventTrigger("keepOutAlgae").onTrue(
      new InstantCommand(() -> arm.setArmRoller(.4), arm)
    );
    
    NamedCommands.registerCommand("notHaveCoral", new WaitUntilCommand(scorer::notHasCoral));
    NamedCommands.registerCommand("pulseCoral", new PulseScorerCommand(scorer));
    NamedCommands.registerCommand("alignLeft", new AlignToTagCommand(robotDrive, vision, -0.18, .33));
    NamedCommands.registerCommand("alignRight", new AlignToTagCommand(robotDrive, vision, 0.195, 0.33));//0.195
    NamedCommands.registerCommand("alignCenter", new AlignToTagCommand(robotDrive, vision));

    new EventTrigger("pulseCoral").onTrue(new WaitUntilCommand(elevator::atHeight).andThen(
      new PulseScorerCommand(scorer)));

    new EventTrigger("armOut").onTrue(
      new InstantCommand(() -> arm.setArmPosition(90), arm).andThen(() -> arm.setArmRoller(-.4), arm));

    new EventTrigger("armDown+eject").onTrue(
      new InstantCommand(() -> arm.setArmPosition(15), arm).andThen(() -> arm.setArmRoller(.3))
    );

    new EventTrigger("elevatorUpL1Algae + ArmDown").onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1Algae), elevator),
      new InstantCommand(() -> arm.setArmPosition(22), arm).andThen(
      new InstantCommand(() -> arm.setArmRoller(-0.4), arm))
    ));

    new EventTrigger("elevatorUpL3").onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L3))
    );
    new EventTrigger("wait").onTrue(new WaitCommand(1));

    new EventTrigger("elevatorUpL2").onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2))
    );

    new EventTrigger("shootAlgae").onTrue(new InstantCommand(() -> arm.setArmRoller(.3), arm));

    NamedCommands.registerCommand("waitTillHeight", new WaitUntilCommand(elevator::atHeight));
    NamedCommands.registerCommand("ejectLeft", new RunCommand(() -> scorer.ejectBottomLeft(), scorer));
    NamedCommands.registerCommand("ejectRight", new RunCommand(() -> scorer.ejectBottomRight(), scorer));

    NamedCommands.registerCommand("ejectAlgae", new RunCommand(() -> arm.setArmRoller(-.4), arm));
    NamedCommands.registerCommand("ejectCoral", new StartEndCommand(() -> scorer.ejectElevated(), () -> scorer.stopScorer()));
    
    new EventTrigger("elevatorDownL0").onTrue(new InstantCommand(
      () -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator));

    new EventTrigger("elevatorDownL0+armOut").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator),
      new InstantCommand(() -> arm.setArmPosition(90), arm).andThen(new InstantCommand(() -> arm.setArmRoller(-.4), arm))));

    new EventTrigger("elevatorUpL2+armOut").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2), elevator),
      new InstantCommand(() -> arm.setArmPosition(90), arm).andThen(() -> arm.setArmRoller(-.4), arm)));

    new EventTrigger("elevatorDownL0+armDown").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator),
      new InstantCommand(() -> arm.setArmPosition(15), arm).andThen(() -> arm.setArmRoller(-.4), arm)));

    // NamedCommands.registerCommand("ejectAlgae", new InstantCommand(() -> arm.setArmRoller(.3), arm).
    //   andThen(new WaitCommand(1)).andThen(new InstantCommand(() -> arm.setArmRoller(0), arm)));

    new EventTrigger("ejectAlgae").onTrue(new RunCommand(() -> arm.setArmRoller(-.4), arm));

    new EventTrigger("elevatorUpL2Algae+armDown").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2Algae), elevator), 
      new InstantCommand(() -> arm.setArmPosition(26)).andThen(new InstantCommand(() -> arm.setArmRoller(-.4), arm))
    ));

    new EventTrigger("elevatorUpL2Algae").onTrue(
      new InstantCommand(() -> elevator.setPosition(28), elevator));

    new EventTrigger("elevatorDownL0+armHold").onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator), 
        new InstantCommand(() -> arm.setArmPosition(ArmConstants.kFullExtendPosition))
      )
    );
    

    /*---------------------------------------------Only For Side Auton----------------------------------------------- */
    new EventTrigger("initiateIntake").onTrue(new CoralIntakeCommand(scorer));
    
    new EventTrigger("elevatorDownL1").onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1), elevator));
    


    new EventTrigger("armDownOnly").onTrue(
      new InstantCommand(() -> arm.setArmPosition(30)));
      
    new EventTrigger("armDown").onTrue(
      new InstantCommand(() -> arm.setArmPosition(25), arm).andThen(() -> arm.setArmRoller(.3), arm));

    new EventTrigger("armUpL2").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(23)), 
      new InstantCommand(() -> arm.setArmPosition(90), arm).andThen(() -> arm.setArmRoller(-.4), arm))
    );
    
    // Configure the trigger bindings
    configureBindings();

    autoPosition = new SendableChooser<AutoPos>();
    autoPosition.addOption("Left", AutoPos.Left);
    autoPosition.addOption("Center", AutoPos.Center);
    autoPosition.addOption("Right", AutoPos.Right);
    autoPosition.setDefaultOption("Center", AutoPos.Center);
    SmartDashboard.putData("Auto Pos", autoPosition);

    autoChooser = AutoBuilder.buildAutoChooser("Center_1CV3");
    SmartDashboard.putData("Auto Mode", autoChooser);
    autoDelay = SmartDashboard.getNumber("Auto Delay", 0);
  }

  
  private void configureBindings() {
    // elevator.setDefaultCommand(new DefaultElevatorCommand(elevator));
    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));
    // led.setDefaultCommand(new LedCycleCommand(led, scorer));

    driverControllerCommand.a().whileTrue(new AlignToTagCommand(robotDrive, vision));
    // driverControllerCommand.a().whileTrue(new RunCommand(() -> robotDrive.setX()));
    driverControllerCommand.y().whileTrue(new RunCommand(() -> robotDrive.setX()));
    driverControllerCommand.start().onTrue(new InstantCommand(() -> robotDrive.zeroHeading(), robotDrive));
    driverControllerCommand.leftBumper().whileTrue(new AlignToTagCommand(robotDrive, vision, -0.18, 0.33));
    driverControllerCommand.rightBumper().whileTrue(new AlignToTagCommand(robotDrive, vision, 0.195, 0.33));

    // coPilotSecondControllerCommand.button(9).whileTrue(new StartEndCommand(() -> winch.openTrap(), () -> winch.stopTrap()));
    // coPilotSecondControllerCommand.button(10).whileTrue(new StartEndCommand(() -> winch.closeTrap(), () -> winch.stopTrap()));

    // driverControllerCommand.a().whileTrue(robotDrive.sysIdQuasistatic(Direction.kForward));
    // driverControllerCommand.y().whileTrue(robotDrive.sysIdQuasistatic(Direction.kBackward)));

    // driverControllerCommand.leftBumper().whileTrue(robotDrive.sysIdDynamic(Direction.kForward));
    // driverControllerCommand.rightBumper().whileTrue(robotDrive.sysIdDynamic(Direction.kBackward));


    // driverControllerCommand.a().whileTrue(new InstantCommand(() -> winch.openTrap()));
    // driverControllerCommand.b().whileTrue(new InstantCommand(() -> winch.closeTrap()));

    // coPilotControllerCommand.y().whileTrue(new InstantCommand(() -> elevator.incremPos(), elevator));
    // coPilotControllerCommand.b().onTrue(new InstantCommand(() -> arm.incremPos(), arm));
    // coPilotControllerCommand.x().onTrue(new InstantCommand(() -> arm.decremPos(), arm));
    // coPilotControllerCommand.a().whileTrue(new InstantCommand(() -> elevator.decremPos(), elevator));

    coPilotControllerCommand.b().onTrue(new InstantCommand(() -> arm.setArmPosition(90), arm));
    coPilotControllerCommand.y().whileTrue(new StartEndCommand(() -> arm.setArmRoller(0.3), () -> arm.setArmRoller(0)));
    coPilotControllerCommand.x().whileTrue(new StartEndCommand(() -> arm.setArmRoller(-0.40), () -> arm.setArmRoller(0)));
    coPilotControllerCommand.a().onTrue(new InstantCommand(() -> arm.setArmPosition(18), arm));
    coPilotControllerCommand.start().onTrue(new InstantCommand(() -> arm.setArmPosition(170), arm));
    coPilotControllerCommand.back().whileTrue(new StartEndCommand(() -> arm.setArmRoller(-0.40), () -> arm.setArmRoller(0)));


    new JoystickButton(copilotController, Button.kLeftBumper.value).whileTrue(new StartEndCommand(() -> scorer.ejectBottomLeft(), 
      () -> scorer.stopScorer()));
    new JoystickButton(copilotController, Button.kRightBumper.value).whileTrue(new StartEndCommand(() -> scorer.ejectBottomRight(), 
      () -> scorer.stopScorer()));

    new Trigger(this::leftTrigger).whileTrue(new CoralIntakeCommand(scorer));
    // new Trigger(this::rightTrigger).whileTrue(new PulseScorerCommand(scorer));
    new Trigger(this::rightTrigger).whileTrue(new StartEndCommand(() -> scorer.ejectElevated(), () -> scorer.stopScorer()));
    new Trigger(this::rightTrigger).onTrue(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new StartEndCommand(() -> scorer.ejectElevated(), () -> scorer.stopScorer(), scorer)
                    .withTimeout(1.0), 
                new InstantCommand(() -> LedSubsystem.setAllianceSolid())
            ),
            new InstantCommand(() -> LedSubsystem.setAllianceSolid()),
            scorer::hasCoral
        )
    );

    new Trigger(this::R1Up).whileTrue(new PulseScorerCommand(scorer));
    new Trigger(this::R1Down).whileTrue(new StartEndCommand(() -> scorer.setBothScorer(-.035), () -> scorer.stopScorer()));

    // new Trigger(this::R1Left).whileTrue(new InstantCommand(() -> arm.incremPos(), arm));
    // new Trigger(this::R1Right).whileTrue(new InstantCommand(() -> arm.decremPos(), arm));

    // new Trigger(this::R1Up).whileTrue(new InstantCommand(() -> elevator.incremPos(), elevator));
    // new Trigger(this::R1Down).whileTrue(new InstantCommand(() -> elevator.decremPos(), elevator));

    coPilotSecondControllerCommand.axisLessThan(4, -0.25).onTrue(new InstantCommand(() -> elevator.incremPos(), elevator));
    coPilotSecondControllerCommand.axisGreaterThan(4, 0.25).onTrue(new InstantCommand(() -> elevator.decremPos(), elevator));
    coPilotSecondControllerCommand.button(1).onTrue(new InstantCommand(() -> arm.incremPos(), arm));
    coPilotSecondControllerCommand.button(3).onTrue(new InstantCommand(() -> arm.decremPos(), arm));

    coPilotSecondControllerCommand.button(2).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1Algae)), 
      new InstantCommand(() -> arm.setArmPosition(25), arm)));
    coPilotSecondControllerCommand.button(4).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2Algae)), 
      new InstantCommand(() -> arm.setArmPosition(25), arm)));

    
    // new Trigger(this::R1Left).whileTrue(new StartEndCommand(() -> arm.setArmRoller(-0.40), () -> arm.setArmRoller(0)));
    // new Trigger(this::R1Right).whileTrue(new StartEndCommand(() -> arm.setArmRoller(0.3), () -> arm.setArmRoller(0)));

    // new Trigger(this::R1Up).whileTrue(new InstantCommand(() -> arm.setArmPosition(90), arm));
    // new Trigger(this::R1Down).whileTrue(new InstantCommand(() -> arm.setArmPosition(15), arm));

    new Trigger(elevator::atDangerHeight)
            .onTrue(new InstantCommand(() -> LedSubsystem.setScrollingRainbow()))
            .onFalse(new InstantCommand(() -> LedSubsystem.setAllianceSolid()));

    new Trigger(vision::hasTarget)
      .onTrue(new InstantCommand(() -> LedSubsystem.blinkAllianceSolidFast()))
      .onFalse(new InstantCommand(() -> LedSubsystem.setAllianceSolid()));

    coPilotControllerCommand.povUp().onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L3), elevator));
    coPilotControllerCommand.povDown().onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator));
    coPilotControllerCommand.povRight().onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1), elevator));
    coPilotControllerCommand.povLeft().onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2), elevator));
  }

  private boolean leftTrigger() {
    return copilotController.getRawAxis(2) > 0.75;
  }
  private boolean rightTrigger() {
    return copilotController.getRawAxis(3) > 0.75;
  }
  private boolean R1Down() {
    return copilotController.getRawAxis(5) > 0.75;
  }
  private boolean R1Up() {
    return copilotController.getRawAxis(5) < -0.75;
  }
  private boolean R1Left(){
    return copilotController.getRawAxis(4) < -0.75;
  }
  private boolean R1Right(){
    return copilotController.getRawAxis(4) > 0.75;
  }
  // private boolean L1Down() {
  //   return copilotController.getRawAxis(1) > 0.75;
  // }
  // private boolean L1Up() {
  //   return copilotController.getRawAxis(1) < -0.75;
  // }

  public Command getAutonomousCommand() {

    robotDrive.zeroHeading();
    if (autoPosition.getSelected() == AutoPos.Center) {
      robotDrive.setFieldRelativeOffset(180);
    }
    else if (autoPosition.getSelected() == AutoPos.Left) {
      robotDrive.setFieldRelativeOffset(-135);
    }
    else if (autoPosition.getSelected() == AutoPos.Right) {
      robotDrive.setFieldRelativeOffset(135);
    }
    return new WaitCommand(autoDelay).andThen(autoChooser.getSelected());
  }


  public enum AutoPos{
    Left, Center, Right
  }
}
