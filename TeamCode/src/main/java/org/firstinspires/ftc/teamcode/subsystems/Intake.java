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
    public final ColorSensor colorSensor;

//    public final BeamBreak beamBreak;

    private static final double armPowerScale = 0.5;
    private static final double servoScale = 0.01;
    private double currentServoPosition = 0.5;
    private static final int armTicks = 10; //idk our encoder resolution

    public Intake (HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.ARM_MOTOR);
        servo = hardwareMap.get(Servo.class, Constants.IntakeConstants.ARM_SERVO);
        colorSensor = hardwareMap.get(ColorSensor.class, Constants.IntakeConstants.ARM_SENSOR);
//        beamBreak = hardwareMap.get(BeamBreak.class, Constants.IntakeConstants.BEAM_BREAK);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void servoControl (Gamepad gamepad2){
        if (gamepad2.right_trigger > 0.1) {
            currentServoPosition += servoScale;
        }
        if (gamepad2.right_trigger > 0.1) {
            currentServoPosition -= servoScale;
        }

        currentServoPosition = Math.min(Math.max(currentServoPosition, 0.0), 1.0);
        servo.setPosition(currentServoPosition);
    }

    public void sampleGrabbed () {

    }
    public void armMotorControl (Gamepad gamepad2){
        int currentPosition = armMotor.getCurrentPosition();
        int targetPosition = currentPosition;

        if (Math.abs(gamepad2.right_stick_y) > 0.1){
            int degreeChange = (int) (gamepad2.right_stick_y * 5);
            targetPosition = currentPosition + (degreeChange * armTicks);
        }

        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(armPowerScale);
    }

    public void update(Gamepad gamepad2){
        armMotorControl(gamepad2);
        servoControl(gamepad2);
    }

    public boolean isArmBusy(){
        return armMotor.isBusy();
    }

    public int getArmPosition(){
        return armMotor.getCurrentPosition();
    }

    public void stop() {
        armMotor.setPower(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo.setPosition(0);
    }

}
