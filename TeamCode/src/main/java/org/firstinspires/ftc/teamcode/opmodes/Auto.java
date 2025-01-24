//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.sequences.GameSequence;
//
//@Autonomous(name = "Auto", group = "Drive")
//public class Auto extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize hardware and game sequence
//        RobotHardware robot = new RobotHardware(this); // Assuming RobotHardware handles hardware mapping
//        GameSequence gameSequence = new GameSequence(robot.hardwareMap, this);
//        // Wait for the start signal
//        waitForStart();
//        // Execute the game sequence during autonomous period
//        if (opModeIsActive()) {
//            gameSequence.execute();
//        }
//        // Clean up resources at the end of the autonomous period
//        gameSequence.stop();
//    }
//}
