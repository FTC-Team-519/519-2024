package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.DriveForward;
import org.firstinspires.ftc.teamcode.util.events.processors.LinearProcessor;
import org.firstinspires.ftc.teamcode.util.events.ZeroArmAngle;

@Disabled
@Autonomous(name = "DriveToObservation", group="Linear OpMode")
public class DriveToObservation extends OpModeBase {
    private LinearProcessor linProcess = null;

    @Override
    public void init() {
        super.init();
        this.linProcess = new LinearProcessor(new DriveForward(12.0, this.robot));
        linProcess.init();
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }else {
            linProcess.shutdown();
        }
    }
}
