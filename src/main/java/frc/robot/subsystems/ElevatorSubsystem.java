package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase{

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pid;

    private final double kp =  0.1,kI = 0.0,kD = 0.0;

    public ElevatorSubsystem(int motorPort){
        motor = new CANSparkMax(motorPort,MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = new PIDController(kp,kI,kD);
        pid.setTolerance(0.01);

    }   
    
    public void setManualSpeed(double speed){
        motor.set(speed);
    }

    public void stop(){
        motor.set(0);
    }

    public void SetTargetPosition(){
        double output = pid.calculate(getPosition(),positionMeters);
        motor.set(output);
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}
