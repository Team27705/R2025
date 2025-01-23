package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Sensor: Generic Switch", group = "Sensor")
//@Disabled
public class BeamBreak extends LinearOpMode {
    private final DigitalChannel genericSwitch;
    private boolean switchState;

    public BeamBreak () {
        /*
         * Initialize the hardware
         */

        genericSwitch = hardwareMap.digitalChannel.get("switch");
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            beamUpdate();
        }
    }

    public void beamUpdate (){
        boolean isItOpen = genericSwitch.getState();


        if (isItOpen) {
            switchState = true;
        } else {
            switchState = false;
        }
        telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
        telemetry.addData("state", ":  " + switchState);
        telemetry.update();

    }
    public boolean beamState(){
        return switchState;
    }

}