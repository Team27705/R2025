package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public final DcMotor armMotor;
    public final Servo servo;

    public Intake (HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.ARM_MOTOR);
        servo = hardwareMap.get(Servo.class, Constants.IntakeConstants.ARM_SERVO);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);

    }

    public void servoControl (){

    }
    public void armMotorControl (){

    }
}
