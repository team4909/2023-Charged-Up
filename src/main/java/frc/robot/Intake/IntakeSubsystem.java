package frc.robot.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem m_instance; 
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    double m_WristSetpoint = 0;

    private CANSparkMax m_wristRight;
    private CANSparkMax m_wristLeft;

    private CANSparkMax m_frontRoller;
    private CANSparkMax m_backRoller;

    private SparkMaxPIDController m_wristPIDController;
    
    public static IntakeSubsystem getInstance(){
        return m_instance = (m_instance == null) ? new IntakeSubsystem() : m_instance;
    }

    private IntakeSubsystem(){
        m_wristRight = new CANSparkMax(3, MotorType.kBrushless);
        m_wristLeft = new CANSparkMax(2, MotorType.kBrushless);
        
        
        m_frontRoller = new CANSparkMax(1, MotorType.kBrushless);
        m_backRoller = new CANSparkMax(4, MotorType.kBrushless);
       
        m_wristLeft.restoreFactoryDefaults();
        m_wristRight.restoreFactoryDefaults(); 
        
        m_frontRoller.restoreFactoryDefaults();
        m_backRoller.restoreFactoryDefaults(); 

        m_wristPIDController = m_wristLeft.getPIDController();


        m_wristRight.follow(m_wristLeft);
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;
    
        m_wristPIDController.setP(kP);
        m_wristPIDController.setI(kI);
        m_wristPIDController.setD(kD);
        m_wristPIDController.setIZone(kIz);
        m_wristPIDController.setFF(kFF);
        m_wristPIDController.setOutputRange(kMinOutput, kMaxOutput);
    
     
    }
    
    

    public void setWristSetpoint(double distance) { 
        
        m_wristPIDController.setReference(distance, CANSparkMax.ControlType.kPosition);
        m_WristSetpoint = distance;

    }

    public void setRollerSpeed(double frontRollerSpeed, double backRollerSpeed){
        if (frontRollerSpeed > 1) {
            frontRollerSpeed = 1;
        }
        else if (frontRollerSpeed < -1) {
            frontRollerSpeed = -1;
        }
        if (backRollerSpeed > 1) {
            backRollerSpeed = 1;
        }
        else if (backRollerSpeed < -1) {
            backRollerSpeed = -1;
        }

        m_frontRoller.set(frontRollerSpeed);
        m_backRoller.set(backRollerSpeed);
    }


    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double encPosition = m_wristLeft.getEncoder().getPosition();
      double positionError = encPosition - m_WristSetpoint;
      
      
      
      SmartDashboard.putNumber("wrist error", positionError);
      SmartDashboard.putNumber("WristSetpoint", m_WristSetpoint);
      SmartDashboard.putNumber("left wrist encoderPosition", encPosition);
    }


}
