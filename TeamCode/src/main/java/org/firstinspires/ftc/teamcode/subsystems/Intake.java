package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.BeamBreak;
public class Intake {

    public final DcMotor armMotor;
    public final Servo servo;
//    public final ColorSensor colorSensor;

//    public final BeamBreak beamBreak;

    private static final double armPowerScale = 0.5;

    private double currentServoPosition = 0;
    private static final double servoMin = 0.0;
    private static final double servoMax = 1.0;
    private static final int armTicks = 10; //idk our encoder resolution

    public Intake (HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.ARM_MOTOR);
        servo = hardwareMap.get(Servo.class, Constants.IntakeConstants.ARM_SERVO);
//        colorSensor = hardwareMap.get(ColorSensor.class, Constants.IntakeConstants.ARM_SENSOR);
//        beamBreak = hardwareMap.get(BeamBreak.class, Constants.IntakeConstants.BEAM_BREAK);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo.setPosition(0);
    }

    public void servoControl (double increment){
        currentServoPosition += increment;

        currentServoPosition = Math.min(Math.max(currentServoPosition, servoMin), servoMax);

        servo.setPosition(currentServoPosition);
    }

    public void resetServo (){

    }


    public void armMotorControl (double power){
        double currentPower = power;
        armMotor.setPower(currentPower);
    }

    public boolean isArmBusy(){
        return armMotor.isBusy();
    }

    public int getArmPosition(){
        return armMotor.getCurrentPosition();
    }
    public void resetMotor (){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    public void dialServo (){
        servo.setPosition(.5);
    }

    public void hold (){
        armMotor.setPower(-1);

    }

    public void stop() {
        armMotor.setPower(0);
        servo.setPosition(0);
    }

    public void resetEncoders(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}