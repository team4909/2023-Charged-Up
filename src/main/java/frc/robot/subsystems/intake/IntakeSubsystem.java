package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    /**
     * States enum for all the different states the intake could be in
     * States: In, Out, and Calibrate (Zeroing)
     */
    public enum IntakeStates {

        IN("IN"),
        CALIBRATE("CALIBRATE"),
        CUBE_IN("CUBE_IN"),
        CUBE_SPIT("CUBE_SPIT"),
        CUBE_INN("CUBE_INN"),
        CUBE_INNN("CUBE_INNN"),
        CONE_IN("CONE_IN"),
        CONE_INN("CONE_INN"),
        CONE_SPIT("CONE_SPIT"),
        HANDOFF("HANDOFF");

        String stateName;

        private IntakeStates(String name) {
            this.stateName = name;
        }

        public String toString() {
            return stateName;
        }
    }

    private IntakeStates m_currentState;
    private IntakeStates m_lastState;

    private static IntakeSubsystem m_instance;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    final double intakeSpeed = 0.5;

    private final CANSparkMax m_hingeRight;
    private final CANSparkMax m_hingeLeft;
    private double m_hinge_setpoint;

    private CANSparkMax m_frontRoller;
    private CANSparkMax m_backRoller;

    double frontRollerReduction;
    double backRollerReduction;
    private SparkMaxPIDController m_positionController;

    public static IntakeSubsystem getInstance() {
        return m_instance = (m_instance == null) ? new IntakeSubsystem() : m_instance;
    }

    private IntakeSubsystem() {
        m_currentState = IntakeStates.IN;
        m_hingeRight = new CANSparkMax(3, MotorType.kBrushless);
        m_hingeLeft = new CANSparkMax(2, MotorType.kBrushless);

        m_frontRoller = new CANSparkMax(1, MotorType.kBrushless);
        m_backRoller = new CANSparkMax(4, MotorType.kBrushless);

        m_hingeLeft.restoreFactoryDefaults();
        m_hingeRight.restoreFactoryDefaults();

        m_frontRoller.restoreFactoryDefaults();
        m_backRoller.restoreFactoryDefaults();

        m_hingeRight.follow(m_hingeLeft);
        kP = 0.02;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.0;
        kMaxOutput = 1;
        kMinOutput = -0.1;

        m_positionController = m_hingeLeft.getPIDController();

        m_positionController.setP(kP);
        m_positionController.setI(kI);
        m_positionController.setD(kD);
        m_positionController.setIZone(kIz);
        m_positionController.setFF(kFF);
        m_positionController.setOutputRange(kMinOutput, kMaxOutput);

        m_hingeLeft.setSmartCurrentLimit(40, 40);
        m_hingeLeft.getEncoder().setPositionConversionFactor(IntakeConstants.DEGREES_PER_TICK);
        m_hingeLeft.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double encPosition = m_hingeLeft.getEncoder().getPosition();
        double positionError = encPosition - m_hinge_setpoint;

        SmartDashboard.putNumber("Hinge error", positionError);
        SmartDashboard.putNumber("Hinge Setpoint", m_hinge_setpoint);
        SmartDashboard.putNumber("Hinge Encoder Position", encPosition);
        SmartDashboard.putNumber("Hinge output", m_hingeLeft.getAppliedOutput());
        SmartDashboard.putString("Intake State", m_currentState == null ? "null" : m_currentState.toString());
        SmartDashboard.putNumber("Hinge Current", m_hingeLeft.getOutputCurrent());
        stateMachine();

        // if (Math.abs(positionError) < 0.5) {
        // m_positionController.setReference(0.0, ControlType.kDutyCycle);
        // }
    }

    private void stateMachine() {
        if (!m_currentState.equals(m_lastState)) {
            switch (m_currentState) {
                case CUBE_IN:
                    // PID Refrence set to out setpoint, ~30
                    m_hinge_setpoint = 5;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(intakeSpeed);
                    m_backRoller.set(-intakeSpeed / 6.0);
                    break;

                case CUBE_SPIT:
                    m_hinge_setpoint = 10;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(-intakeSpeed);
                    m_backRoller.set(intakeSpeed);
                    break;
                case CUBE_INN:
                    m_hinge_setpoint = 10.5;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(intakeSpeed);
                    m_backRoller.set(intakeSpeed / 2.0);
                    break;
                case CUBE_INNN:
                    m_hinge_setpoint = 10.5;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(intakeSpeed);
                    m_backRoller.set(0);
                    break;
                case CONE_IN:
                    m_hinge_setpoint = 11;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(intakeSpeed);
                    m_backRoller.set(intakeSpeed / 2.0);
                    break;
                case CONE_INN:
                    m_hinge_setpoint = 11;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(intakeSpeed);
                    m_backRoller.set(-intakeSpeed / 2.0);
                    break;
                case CONE_SPIT:
                    m_hinge_setpoint = 100d;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(intakeSpeed);
                    m_backRoller.set(-intakeSpeed);
                    break;
                case CALIBRATE:
                    // Calls the calibrate method
                    calibrateIntake();
                    break;
                case IN:
                    m_hinge_setpoint = 100d;
                    setSetpoint(m_hinge_setpoint);
                    m_frontRoller.set(0);
                    m_backRoller.set(0);
                    break;
                case HANDOFF:
                    if (m_lastState.toString() == "CUBE_IN") {
                        m_hinge_setpoint = 66;
                        setSetpoint(m_hinge_setpoint);
                        m_frontRoller.set(0.2);
                        m_backRoller.set(-.05);
                    } else if (m_lastState.toString() == "CONE_IN") {
                        m_hinge_setpoint = 66;
                        setSetpoint(m_hinge_setpoint);
                        m_frontRoller.set(0.2);
                        m_backRoller.set(0.1);
                    }
                    break;
            }

            m_lastState = m_currentState;
        }

    }

    private void setSetpoint(double setpoint) {
        double ff = calcFF(m_hingeLeft.getEncoder().getPosition());
        m_positionController.setReference(setpoint, ControlType.kPosition, 0,
                ff);
    }

    private void calibrateIntake() {
        // Runs the intake backwards for .75 seconds, and then sets the encoder position
        // to 0
        new RunCommand(() -> {
            // m_positionController.setReference(-.2, ControlType.kDutyCycle);
            m_hingeLeft.setSmartCurrentLimit(5, 40);
            m_hingeLeft.set(.1);
        }, this)
                .withTimeout(0.75)
                .andThen(new InstantCommand(() -> {
                    m_hingeLeft.getEncoder().setPosition(110d);
                    // m_positionController.setReference(m_hinge_setpoint, ControlType.kPosition);
                    m_currentState = IntakeStates.IN;
                    m_hingeLeft.setSmartCurrentLimit(40, 40);
                })).schedule();
    }

    public void intakeZero() {
        m_currentState = IntakeStates.CALIBRATE;
    }

    public void cubeIn() {
        m_currentState = IntakeStates.CUBE_IN;
    }

    public void cubeSpit() {
        m_currentState = IntakeStates.CUBE_SPIT;
    }

    public void cubeInn() {
        m_currentState = IntakeStates.CUBE_INN;
    }

    public void cubeInnn() {
        m_currentState = IntakeStates.CUBE_INNN;
    }

    public void coneIn() {
        m_currentState = IntakeStates.CONE_IN;
    }

    public void coneInn() {
        m_currentState = IntakeStates.CONE_INN;
    }

    public void coneSpit() {
        m_currentState = IntakeStates.CONE_SPIT;
    }

    public void intakeIn() {
        m_currentState = IntakeStates.IN;
    }

    public void handOff() {
        m_currentState = IntakeStates.HANDOFF;
    }

    private double calcFF(double theta) {
        double ff = IntakeConstants.kG * Math.cos(theta);
        SmartDashboard.putNumber("Intake FF", ff);
        return ff;
    }
}
