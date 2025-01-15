package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.IntakePiece;
import org.firstinspires.ftc.teamcode.util.events.LinearProcessor;
import org.firstinspires.ftc.teamcode.util.events.ZeroArmAngle;

@TeleOp(name = "TestIntake", group="Linear OpMode")
//@Disabled
public class TestIntake extends OpModeBase {
    private boolean buttonAPressed = false;
    private LinearProcessor linProcess = null;

    @Override
    public void init() {
        super.init();
        this.linProcess = new LinearProcessor(new IntakePiece(this.robot,1.0, true), new IntakePiece(this.robot,1.0, false));
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }
        linProcess.shutdown();

        if (!this.buttonAPressed && gamepad1.a){
            return;
        }

        this.buttonAPressed = gamepad1.a;
    }
}
