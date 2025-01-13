package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ControllerConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

public class Drivetrain {
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final FtcDashboard dashboard;

    public Drivetrain(HardwareMap hardwareMap) {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, DriveConstants.LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotor.class, DriveConstants.RIGHT_FRONT_MOTOR);

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        resetEncoders();

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
    }

    public void setPower(double drive, double turn) {
        // Combine drive and turn
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize powers to stay within [-1, 1]
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Apply speed limits from constants
        leftPower = clamp(leftPower, DriveConstants.MIN_SPEED, DriveConstants.MAX_SPEED);
        rightPower = clamp(rightPower, DriveConstants.MIN_SPEED, DriveConstants.MAX_SPEED);

        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);

        // Send data to dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left Motor Power", leftPower);
        packet.put("Right Motor Power", rightPower);
        packet.put("Drive Power", drive);
        packet.put("Turn Power", turn);
        packet.put("Left Encoder", leftFront.getCurrentPosition());
        packet.put("Right Encoder", rightFront.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
    }

    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public void stop() {
        setPower(0, 0);
    }

    public int getLeftPosition() {
        return leftFront.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightFront.getCurrentPosition();
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}