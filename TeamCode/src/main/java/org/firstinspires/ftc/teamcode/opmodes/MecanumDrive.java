package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants.ControllerConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Vision.TagPose;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/**
 * Gamepad 1(A) controls driving
 * Gamepad 2(B) controls intake
 *
 * */

@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumDrive extends LinearOpMode {
    private RobotHardware robot;
    private boolean rbPressed = false;
    private boolean lbPressed = false;
    private long lastSpeedChangeTime = 0;
    private static final long debounceTime = 250;

    private boolean armMoving = false;
    private long lastArmMoveTime = 0;
    private boolean servoMoving = false;
    private long lastServoMoveTime = 0;
    private static final long armDebounceTime = 100;
    private static final long servoDebounceTime = 50;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Combine telemetry with dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.init();

        // Enable camera stream in Driver Station app
        if (robot.vision.getVisionPortal() != null) {
            robot.vision.getVisionPortal().setProcessorEnabled(robot.vision.getAprilTagProcessor(), true);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Left Stick = Drive + Turn");
        telemetry.update();

        waitForStart();
        
        while (opModeIsActive()) {
            handleDriveControls();
            handleSpeedControls();
            handleUtilityControls();
            handleIntakeControls();
            updateTelemetry();
        }
        robot.drivetrain.stop();
    }

    private void handleDriveControls() {
        // Get joystick values
        double drive = gamepad1.left_stick_y;   // Forward/back
        double strafe = -gamepad1.left_stick_x;   // Left/right
        double turn = -gamepad1.right_stick_x;    // Turning

        // Apply deadband
        if (Math.abs(drive) < ControllerConstants.STICK_DEADBAND) drive = 0;
        if (Math.abs(strafe) < ControllerConstants.STICK_DEADBAND) strafe = 0;
        if (Math.abs(turn) < ControllerConstants.STICK_DEADBAND) turn = 0;

        // Apply speed multiplier and drive
        robot.drivetrain.setMecanumPower(
                drive * DriveConstants.SPEED_MULTIPLIER,
                strafe * DriveConstants.SPEED_MULTIPLIER,
                turn * DriveConstants.SPEED_MULTIPLIER
        );
    }

    private void handleIntakeControls() {
        double turn = gamepad2.right_stick_y;

        long currentTime = System.currentTimeMillis();

        if (Math.abs(turn) < ControllerConstants.STICK_DEADBAND) {turn = 0;}
        //handle debouncing

        robot.intake.armMotorControl(turn);

        if (gamepad2.right_trigger > .1 || gamepad2.left_trigger > 0.1){
            if (!servoMoving || (currentTime - lastServoMoveTime > servoDebounceTime)){
                robot.intake.servoControl(gamepad2);
                servoMoving = true;
                lastServoMoveTime = currentTime;
            }
        } else {
            servoMoving = false;
        }
    }

    private void handleSpeedControls() {
        // Adjust speed multiplier with bumpers
        // need to implement debouncing
        long currentTime = System.currentTimeMillis();

        if (gamepad1.right_bumper && !rbPressed && currentTime - lastSpeedChangeTime > debounceTime) {
            if (DriveConstants.SPEED_MULTIPLIER < 1.0){
                DriveConstants.SPEED_MULTIPLIER += 0.25;
                lastSpeedChangeTime = currentTime;
            }
            rbPressed = true;
        } else if (!gamepad1.right_bumper){
            rbPressed = false;
        }

        if (gamepad1.left_bumper && !lbPressed && currentTime - lastSpeedChangeTime > debounceTime) {
            if (DriveConstants.SPEED_MULTIPLIER > 0.25){
                DriveConstants.SPEED_MULTIPLIER -= 0.25;
                lastSpeedChangeTime = currentTime;
            }
            lbPressed = true;
        } else if (!gamepad1.left_bumper){
            lbPressed = false;
        }
    }

    private void handleUtilityControls() {
        // Reset encoders with Y button
        if (gamepad1.y) {
            robot.drivetrain.resetEncoders();
            robot.intake.resetEncoders();
        }
        if (gamepad1.b) {
            robot.drivetrain.stop();
        }
    }
    // http://192.168.43.1:8080/dash
    private void updateTelemetry() {
        telemetry.addData("=== DRIVER CONTROLS ===", "");
        telemetry.addData("Drive Power", "%.2f", -gamepad1.left_stick_y * DriveConstants.SPEED_MULTIPLIER);
        telemetry.addData("Turn Power", "%.2f", gamepad1.left_stick_x * DriveConstants.SPEED_MULTIPLIER);
        telemetry.addData("Speed Multiplier", "%.2f", DriveConstants.SPEED_MULTIPLIER);
        // Add AprilTag pose information
        TagPose pose = robot.vision.getRelativePose();
        telemetry.addData("\n=== APRILTAG DATA ===", "");
        if (pose != null) {
            telemetry.addData("Tag X", "%.2f in", pose.x);
            telemetry.addData("Tag Y", "%.2f in", pose.y);
            telemetry.addData("Tag Z", "%.2f in", pose.z);
            telemetry.addData("Tag Heading", "%.2f°", pose.heading);
            telemetry.addData("Tag ID", "%d", pose.tagID);
            telemetry.addData("Distance from Tag", "%.2f in",
                    Math.sqrt(pose.x * pose.x + pose.y * pose.y));
        } else {
            telemetry.addData("Tag Status", "No tag detected");
        }
        telemetry.addData("\n=== ENCODER DATA ===", "");
        telemetry.addData("\n=== CONTROLS ===", "");
        telemetry.addData("Drive", "Left Stick = Move + Turn");
        telemetry.addData("Speed", "Bumpers = Adjust Speed");
        telemetry.addData("Utility", "Y = Reset Encoders");
        telemetry.update();
    }
}
