package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

public class Drivetrain {
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;
    private final FtcDashboard dashboard;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    public Drivetrain(HardwareMap hardwareMap) {
        // Initialize all four motors
        leftFront = hardwareMap.get(DcMotor.class, DriveConstants.LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotor.class, DriveConstants.RIGHT_FRONT_MOTOR);
        leftBack = hardwareMap.get(DcMotor.class, DriveConstants.LEFT_BACK_MOTOR);
        rightBack = hardwareMap.get(DcMotor.class, DriveConstants.RIGHT_BACK_MOTOR);

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

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

    public void setMecanumPower(double drive, double strafe, double turn) {
        leftFrontPower = drive + strafe + turn;
        rightFrontPower = drive - strafe - turn;
        leftBackPower = drive - strafe + turn;
        rightBackPower = drive + strafe - turn;

        // Normalize powers to ensure they're within [-1, 1]
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Dashboard telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left Front Power", leftFrontPower);
        packet.put("Right Front Power", rightFrontPower);
        packet.put("Left Back Power", leftBackPower);
        packet.put("Right Back Power", rightBackPower);
        dashboard.sendTelemetryPacket(packet);
    }

    //to decelerate the motors to zero

    public void stop() {
        setMecanumPower(0, 0, 0);
    }

//    public void test(){
//        leftFront.setTargetPosition(360);
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftFront.setPower(.1);
//
//    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void MotorTelemtry(Telemetry telemetry) {
        telemetry.addData("\nLeft Front Motor", leftFront.getCurrentPosition());
        telemetry.addData("\nLeft Back Motor", leftBack.getCurrentPosition());
        telemetry.addData("\nRight Front Motor", rightFront.getCurrentPosition());
        telemetry.addData("\nRight Back Motor", rightBack.getCurrentPosition());

    }

    public DcMotor getLeftFront () {return leftFront;}
    public DcMotor getLeftBack () {return leftBack;}

    public DcMotor getRightFront () {return rightFront;}

    public DcMotor getRightBack () {return  rightBack;}
}