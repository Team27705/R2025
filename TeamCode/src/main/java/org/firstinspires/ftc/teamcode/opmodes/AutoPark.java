package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous(name = "Auto Park", group = "Autonomous")
public class AutoPark extends LinearOpMode {
    private Drivetrain drivetrain;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();


    private static final double ROTATE_SPEED = 0.25;
    private static final double MOVE_SPEED = 0.25;
    private static final double ROTATE_TIME = 1.5;
    private static final double MOVE_TIME = 3.5;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        imu = drivetrain.getImu();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        rotateRobot(90, ROTATE_SPEED);

        moveForward(MOVE_TIME, MOVE_SPEED);

        drivetrain.stop();

        telemetry.addData("Status", "Parking Complete");
        telemetry.update();
    }


    private void rotateRobot(double targetAngle, double speed) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double startAngle = orientation.getYaw(AngleUnit.DEGREES);
        double endAngle = startAngle + targetAngle;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < ROTATE_TIME) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (currentAngle < endAngle) {
                drivetrain.setMecanumPower(0, 0, speed);
            } else {
                break;
            }
        }
        drivetrain.stop();
    }


    private void moveForward(double time, double speed) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            drivetrain.setMecanumPower(speed, 0, 0);
        }
        drivetrain.stop();
    }
}