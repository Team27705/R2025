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

    public final BeamBreak beamBreak;

    private static final double armPowerScale = 0.5;
    private static final double servoScale = 0.01;
    private double currentServoPosition = 0.5;
    private static final int armTicks = 10; //idk our encoder resolution

    public Intake (HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.ARM_MOTOR);
        servo = hardwareMap.get(Servo.class, Constants.IntakeConstants.ARM_SERVO);
        colorSensor = hardwareMap.get(ColorSensor.class, Constants.IntakeConstants.ARM_SENSOR);
        beamBreak = hardwareMap.get(BeamBreak.class, Constants.IntakeConstants.BEAM_BREAK);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }



    public void servoControl (Gamepad gamepad2){
        if (gamepad2.right_bumper) {
            currentServoPosition += servoScale;
        }
        if (gamepad2.left_bumper) {
            currentServoPosition -= servoScale;
        }
        currentServoPosition = Math.min(Math.max(currentServoPosition, 0.0), 1.0);
        servo.setPosition(currentServoPosition);
    }

    public void sampleGrabbed () {
        if (beamBreak.beamState()) {

        }
    }
    public void armMotorControl (double power){
        double currentPower = power;
        if (power > 1){
            currentPower /= Math.abs(currentPower);
        }
        if (power < -1) {
            currentPower /= Math.abs(currentPower);
        }
        armMotor.setPower(currentPower);
    }

    public void update(){

    }

    public boolean isArmBusy(){
        return armMotor.isBusy();
    }

    public int getArmPosition(){
        return armMotor.getCurrentPosition();
    }

    public void stop() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
        servo.setPosition(0);
    }

    public void resetEncoders(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}