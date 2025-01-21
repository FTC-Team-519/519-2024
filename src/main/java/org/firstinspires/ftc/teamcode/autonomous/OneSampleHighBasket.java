package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name = "OneSampleHighBasket", group="Linear OpMode")
public class OneSampleHighBasket extends OpModeBase {
    private LinearProcessor linProcess = null;

    private final double degreesToFirstSample = 128;
    private final double degreesToSecondSample = 100;

    @Override
    public void init() {
        ///robot.resetSpoolEncoders();
        super.init();
        this.linProcess = new LinearProcessor(
                // Put preloaded sample in highbasket
            new ParallelRaceProcessor(
                new ArmStall(17, this.robot),
                new LinearProcessor(
                    new DriveSideways(6.0, this.robot),
                    new DriveForward(7.0, this.robot),
                    new ParallelProcessor(
                        new Rotate(-30.0, false, 1.0, this.robot),
                        new ArmExtension(-4000, 1, this.robot)
                    ),
                        new DriveSideways(7.0, this.robot),
                        new DriveForward(2.0, this.robot),


                    new IntakePiece(this.robot,1.0, false),

                    new DriveForward(-2.0, this.robot)
                    //new ArmExtension(-50, this.robot)
                )
            ),
           // pick up first sample and retract arm
           new LinearProcessor(
                   new Rotate(degreesToFirstSample, true, 0.75, this.robot),
                   new ParallelRaceProcessor(
                           new ParallelProcessor(
                                   new ArmRotation(70.0, this.robot)

                           ),
                           new ArmExtension(-50, 1, this.robot)
                ),
                new ParallelRaceProcessor(
                    new ArmExtension(-3512, 0.2, this.robot),
                    new IntakePiece(this.robot, 1.0, true)
                ),
                // Time out for retrackt
                new ParallelRaceProcessor(
                    new ArmExtension(-40, 1, this.robot),
                    new Timer(2000)
                )
           ),

           new ParallelRaceProcessor(
               new ArmStall(17, this.robot),
               new LinearProcessor(
                   new Rotate(degreesToFirstSample, false, 0.75, this.robot),
                   new ArmExtension(-4000, 1, this.robot),
                   new DriveForward(1.5, this.robot),
                   new IntakePiece(this.robot, 1.0, false)
               )
           ),

            new LinearProcessor(
                new DriveForward(-1.5, this.robot),
                new Rotate(degreesToSecondSample, true, 0.75, this.robot),
                new ParallelRaceProcessor(
                        new ParallelProcessor(
                                new ArmRotation(70.0, this.robot)

                        ),
                        new ArmExtension(-50, 1, this.robot)
                ),
                new ParallelRaceProcessor(
                        new ArmExtension(-3512, 0.2, this.robot),
                        new IntakePiece(this.robot, 1.0, true)
                ),
                new ArmExtension(-50, 1, this.robot)
            )

        );
        this.linProcess.init();
        this.robot.setSpoolMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.setSpoolMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }
//        linProcess.shutdown();
        telemetry.addData("Left Spool Readout: ",robot.getCurrentLeftSpoolPosition());
        // This section should return the current voltage
        telemetry.addData("Potentiometer Output: ",robot.readAbsoluteEncoderValue());
        telemetry.addData("Degree From Back: ",robot.readAbsoluteEncoderValueAsDegrees());
    }
}
