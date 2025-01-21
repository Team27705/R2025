package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

public class Drivetrain {
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;
    private final FtcDashboard dashboard;

    private double leftBackPower;
    private double rightBackPower;
    private double leftFrontPower;
    private double rightFrontPower;

    public Drivetrain(HardwareMap hardwareMap) {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, DriveConstants.LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotor.class, DriveConstants.RIGHT_FRONT_MOTOR);
        leftBack = hardwareMap.get(DcMotor.class, DriveConstants.LEFT_BACK_MOTOR);
        rightBack = hardwareMap.get(DcMotor.class, DriveConstants.RIGHT_BACK_MOTOR);

        // Set motor directions
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        resetEncoders();

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }


    //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

    public void setMecanumPower(double drive, double strafe, double turn) {
        // Combine drive and turn
        double leftFrontPower = drive + turn + strafe;
        double leftBackPower = drive + turn - strafe;
        double rightFrontPower = drive - turn + strafe;
        double rightBackPower = drive - turn - strafe;

        // Normalize powers to stay within [-1, 1]
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)));
        if (maxPower  > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Apply speed limits from constants
        leftFrontPower = clamp(leftFrontPower, DriveConstants.MIN_SPEED, DriveConstants.MAX_SPEED);
        leftBackPower = clamp(leftBackPower, DriveConstants.MIN_SPEED, DriveConstants.MAX_SPEED);
        rightFrontPower = clamp(rightFrontPower, DriveConstants.MIN_SPEED, DriveConstants.MAX_SPEED);
        rightBackPower = clamp(rightBackPower, DriveConstants.MIN_SPEED, DriveConstants.MAX_SPEED);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightBackPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Send data to dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left Front Motor Power", leftFrontPower);
        packet.put("Right Front Motor Power", rightFrontPower);
        packet.put("Left Back Motor Power", leftBackPower);
        packet.put("Right Back Motor Power", rightBackPower);

        packet.put("Left Front Motor Encoder", leftFront.getCurrentPosition());
        packet.put("Right Front Motor Encoder", rightFront.getCurrentPosition());
        packet.put("Left Back Motor Encoder", leftBack.getCurrentPosition());
        packet.put("Right Back Motor Encoder", rightBack.getCurrentPosition());

        packet.put("Drive Power", drive);
        packet.put("Strafe Power", strafe);
        packet.put("Turn Power", turn);


        dashboard.sendTelemetryPacket(packet);
    }


    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public void stop() {
        setMecanumPower(0, 0, 0);
    }


    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void testOperation (){
        leftFront.setTargetPosition(360);
    }
}