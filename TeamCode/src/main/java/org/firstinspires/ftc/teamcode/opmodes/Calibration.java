package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name = "Calibration", group = "testing")

public class Calibration extends LinearOpMode {
    private RobotHardware robot;
    @Override
    public void runOpMode(){
        robot = new RobotHardware(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init();

        if (robot.vision.getVisionPortal() != null) {
            robot.vision.getVisionPortal().setProcessorEnabled(robot.vision.getAprilTagProcessor(), true);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Left Stick = Drive + Turn");
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Left Stick = Drive + Turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            updateTelemetry();
            testMotorRotation();
        }
    }

    public void updateTelemetry(){

    }

    public void testMotorRotation(){
    }

}
