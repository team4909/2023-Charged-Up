package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;
import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Claw.ClawStates;
import frc.robot.subsystems.drivetrain.DefaultDriveCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeStates;

public class RobotContainer {

  private final IntakeSubsystem m_intakeSubsytem = IntakeSubsystem.getInstance();

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final AutoRoutines m_routines = new AutoRoutines();
  private final Elevator m_elevator = Elevator.getInstance();

  private final Arm m_arm = Arm.getInstance();
  private final Claw m_claw = Claw.getInstance();
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public RobotContainer() {
    configureBindings();
    configureSendableChooser();
    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(m_driverController));
  }

  private void configureBindings() {
    // #region Driver Controls
    
        m_drivetrain.setJoystickSuppliers(
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX());

        m_driverController.back().onTrue(m_drivetrain.zeroGyro());
        m_driverController.rightBumper()
                .onTrue(m_drivetrain.setState(DrivetrainStates.PRECISE))
                .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));

        m_driverController.y()
                .onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE, new HashMap<>(Map.of("Angle", 0d))))
                .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
        m_driverController.b()
                .onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE, new HashMap<>(Map.of("Angle", 270d))))
                .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
        m_driverController.a()
                .onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE, new HashMap<>(Map.of("Angle", 180d))))
                .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
        m_driverController.x()
                .onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE, new HashMap<>(Map.of("Angle", 90d))))
                .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
    // m_driverController.leftTrigger() Intake Cube
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_intakeSubsytem.coneSpit()));
    m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_intakeSubsytem.coneIn()))
        // m_driverController.leftTrigger()
        .onFalse(new InstantCommand(() -> m_intakeSubsytem.handOff()));

    m_driverController.povDown().onTrue(new InstantCommand(() -> m_intakeSubsytem.cubeInn()))
        .onFalse(new InstantCommand(() -> m_intakeSubsytem.handOff()));

    m_driverController.povUp().onTrue(new InstantCommand(() -> m_intakeSubsytem.cubeInnn()))
        .onFalse(new InstantCommand(() -> m_intakeSubsytem.handOff()));

    m_driverController.povRight().onTrue(new InstantCommand(() -> m_intakeSubsytem.cubeIn()))
        .onFalse(new InstantCommand(() -> m_intakeSubsytem.handOff()));

    m_driverController.povLeft().onTrue(new InstantCommand(() -> m_intakeSubsytem.coneInn()))
        .onFalse(new InstantCommand(() -> m_intakeSubsytem.handOff()));

    m_driverController.start().onTrue(new InstantCommand(() -> m_intakeSubsytem.intakeZero()));
    m_driverController.x().onTrue(new InstantCommand(() -> m_intakeSubsytem.intakeIn()));

    m_operatorController.povUp().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.TOP)));
    m_operatorController.back().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.HANDOFF_CONE)));
    // m_operatorController.b().onTrue(new InstantCommand(() ->
    // m_arm.setState(ArmStates.HANDOFF_CUBE)));
    m_operatorController.x().onTrue(new InstantCommand(() -> m_claw.setState(ClawStates.OPEN)));
    m_operatorController.y().onTrue(new InstantCommand(() -> m_claw.setState(ClawStates.CLOSED)));

    // Mid Cone Sequence
    m_operatorController.a().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> m_claw.setState(ClawStates.OPEN)),
            new InstantCommand(() -> m_arm.setState(ArmStates.HANDOFF_CONE)),
            new WaitCommand(1),
            new InstantCommand(() -> m_claw.setState(ClawStates.CLOSED)),
            new WaitCommand(1),
            new InstantCommand(() -> m_intakeSubsytem.coneSpit()),
            new WaitCommand(1),
            new InstantCommand(() -> m_arm.setState(ArmStates.TOP)))

    );

    m_operatorController.start().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.DROPPING_FLICK))
        .andThen(new WaitCommand(0.5))
        .andThen(() -> m_claw.setState(ClawStates.OPEN))
        .andThen(new WaitCommand(0.2))
        .andThen(() -> m_elevator.setState(ElevatorStates.RETRACT)));

    m_operatorController.b().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.DROPPING))
        .andThen(new WaitCommand(0.5))
        // .andThen(() -> m_claw.setState(ClawStates.OPEN))
        .andThen(new WaitCommand(0.2))
        .andThen(() -> m_elevator.setState(ElevatorStates.RETRACT)));

    m_operatorController.rightTrigger().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.MID_CONE)));
    m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.TOP)));
    m_operatorController.povDown().onTrue(new InstantCommand(() -> m_claw.setState(ClawStates.CLOSED)));
    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.TOP)));
    m_operatorController.leftTrigger().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.DROPPING)));

    // m_driverController.leftBumper()
    // .onTrue(new RunCommand(() -> m_intakeSubsytem.cubeSpit(), m_intakeSubsytem))
    // .onFalse(new RunCommand(() -> m_intakeSubsytem.handOff(), m_intakeSubsytem));

    // m_driverController.rightTrigger()
    // .onTrue(new RunCommand(() -> m_intakeSubsytem.coneIn(), m_intakeSubsytem))
    // .onFalse(new RunCommand(() -> m_intakeSubsytem.handOff(), m_intakeSubsytem));

    // m_driverController.rightBumper()
    // .onTrue(new RunCommand(() -> m_intakeSubsytem.coneSpit(), m_intakeSubsytem))
    // .onFalse(new RunCommand(() -> m_intakeSubsytem.handOff(), m_intakeSubsytem));
    // m_driverController.x()
    // .onTrue(new RunCommand(() -> m_intakeSubsytem.intakeIn(), m_intakeSubsytem));

    // m_operatorController.rightTrigger().onTrue(new InstantCommand(() ->
    // m_elevator.setState(ElevatorStates.TOP)));
    // m_operatorController.rightBumper().onTrue(new InstantCommand(() ->
    // m_elevator.setState(ElevatorStates.MID_CUBE)));
    // m_operatorController.leftBumper().onTrue(new InstantCommand(() ->
    // m_elevator.setState(ElevatorStates.MID_CONE)));
    // m_operatorController.leftTrigger().onTrue(new InstantCommand(() ->
    // m_elevator.setState(ElevatorStates.RETRACT)));

  }

  public Command getOtherAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.setGyro(180)),
        new InstantCommand(() -> m_elevator.setState(ElevatorStates.TOP)),
        Commands.waitSeconds(2),
        new InstantCommand(() -> m_arm.setState(ArmStates.DROPPING))
            .andThen(new WaitCommand(0.5))
            .andThen(() -> m_claw.setState(ClawStates.OPEN))
            .andThen(new WaitCommand(0.2))
            // .andThen(() -> m_arm.setState(ArmStates.TOP))
            .andThen(() -> m_elevator.setState(ElevatorStates.RETRACT))
    // .andThen(new WaitCommand(3))
    // .andThen(() -> m_drivetrain.traj(PathPlanner.loadPath("StraightPath", 3, 2),
    // false))
    );
    // return new InstantCommand(() ->
    // m_drivetrain.runPath(PathPlanner.loadPath("StraightPath", 3, 2)));
  }

    private void configureSendableChooser() {
        m_chooser.setDefaultOption("Test Auto", m_routines.CHARGE_STATION);
        SmartDashboard.putData(m_chooser);
    }
    
     public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
