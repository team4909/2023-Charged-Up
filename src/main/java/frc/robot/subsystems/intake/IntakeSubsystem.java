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
        CONE_IN("CONE_IN"),
        CONE_SPIT("CONE_SPIT");
       

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
    double m_WristSetpoint = 0;
    final double intakeSpeed = 0.25; 

    private CANSparkMax m_hingeRight;
    private CANSparkMax m_hingeLeft;

    private CANSparkMax m_frontRoller;
    private CANSparkMax m_backRoller;

    private SparkMaxPIDController m_wristPIDController;
    double frontRollerReduction;
    double backRollerReduction;
    private SparkMaxPIDController m_positionController;

    public static IntakeSubsystem getInstance() {
        return m_instance = (m_instance == null) ? new IntakeSubsystem() : m_instance;
    }

    private IntakeSubsystem() {
        m_hingeRight = new CANSparkMax(3, MotorType.kBrushless);
        m_hingeLeft = new CANSparkMax(2, MotorType.kBrushless);

        m_frontRoller = new CANSparkMax(1, MotorType.kBrushless);
        m_backRoller = new CANSparkMax(4, MotorType.kBrushless);

        m_hingeLeft.restoreFactoryDefaults();
        m_hingeRight.restoreFactoryDefaults();

        m_frontRoller.restoreFactoryDefaults();
        m_backRoller.restoreFactoryDefaults();

        m_wristPIDController = m_hingeLeft.getPIDController();

        m_hingeRight.follow(m_hingeLeft);
        kP = 1;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 0.4;
        kMinOutput = -1;

        m_wristPIDController.setP(kP);
        m_wristPIDController.setI(kI);
        m_wristPIDController.setD(kD);
        m_wristPIDController.setIZone(kIz);
        m_wristPIDController.setFF(kFF);
        m_wristPIDController.setOutputRange(kMinOutput, kMaxOutput);

        m_positionController = m_hingeLeft.getPIDController();
    }

    public void setWristSetpoint(double distance) {

        m_wristPIDController.setReference(distance, CANSparkMax.ControlType.kPosition);
        m_WristSetpoint = distance;

    }

    public void setRollerSpeed(double frontRollerSpeed, double backRollerSpeed) {
        if (frontRollerSpeed > 1) {
            frontRollerSpeed = 1;
        } else if (frontRollerSpeed < -1) {
            frontRollerSpeed = -1;
        }
        if (backRollerSpeed > 1) {
            backRollerSpeed = 1;
        } else if (backRollerSpeed < -1) {
            backRollerSpeed = -1;
        }

        m_frontRoller.set(frontRollerSpeed);
        m_backRoller.set(backRollerSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double encPosition = m_hingeLeft.getEncoder().getPosition();
        double positionError = encPosition - m_WristSetpoint;

        SmartDashboard.putNumber("wrist error", positionError);
        SmartDashboard.putNumber("WristSetpoint", m_WristSetpoint);
        SmartDashboard.putNumber("left wrist encoderPosition", encPosition);
        
        
        stateMachine();
    }

    private void stateMachine() {

        if (m_currentState == m_lastState) {
            return; // nothing to do
        }

        switch (m_currentState) {
            case CUBE_IN:
                // PID Refrence set to out setpoint, ~30
                m_positionController.setReference(10, ControlType.kPosition);
                m_frontRoller.set(intakeSpeed);
                m_backRoller.set(-intakeSpeed);
                break;

            case CUBE_SPIT:
                // PID Refrence set to zero
                m_positionController.setReference(10, ControlType.kPosition);
                m_frontRoller.set(-intakeSpeed);
                m_backRoller.set(intakeSpeed);
                break;
            case CONE_IN:
                // PID Refrence set to zero
                m_positionController.setReference(10, ControlType.kPosition);
                m_frontRoller.set(intakeSpeed);
                m_backRoller.set(intakeSpeed);
                break;
            case CONE_SPIT:
                // PID Refrence set to zero
                m_positionController.setReference(10, ControlType.kPosition);
                m_frontRoller.set(intakeSpeed);
                m_backRoller.set(-intakeSpeed);
                break;
            case CALIBRATE:
                // Calls the calibrate method
                calibrateIntake();
                break;
            case IN:
                // PID Refrence set to zero
                m_positionController.setReference(2, ControlType.kPosition);
                m_frontRoller.set(0);
                m_backRoller.set(0);
                break;   
        }

        m_lastState = m_currentState;

    }

    private void calibrateIntake() {
        // Runs the intake backwards for .75 seconds, and then sets the encoder position
        // to 0
        new RunCommand(() -> {
            // m_positionController.setReference(-.2, ControlType.kDutyCycle);
            m_hingeLeft.set(-.2);
        }, this)
                .withTimeout(0.75)
                .andThen(new InstantCommand(() -> {
                    m_hingeLeft.getEncoder().setPosition(0);
                    m_positionController.setReference(0, ControlType.kPosition);
                })).schedule();
    }
    
    public void intakeZero(){
        m_currentState = IntakeStates.CALIBRATE;
    }

    public void cubeIn(){
        m_currentState = IntakeStates.CUBE_IN;
    }

    public void cubeSpit(){
        m_currentState = IntakeStates.CUBE_SPIT;
    }

    public void coneIn(){
        m_currentState = IntakeStates.CONE_IN;
    }

    public void coneSpit(){
        m_currentState = IntakeStates.CONE_SPIT;
    }

    public void intakeIn(){
        m_currentState = IntakeStates.IN;
    }

}
