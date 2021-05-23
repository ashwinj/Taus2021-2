//Run code for the red right position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "RedRight2", group = "Taus")
public class RedRight2 extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 10);
                shoot(-25.0, (2150*28)/60.0);
                toAngle(145, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(0, 1);
                backward(.5, 1, 0);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(-25.0, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 27);//21
                toAngle(-15,1);
                dropWobbleGoal();
                toAngle(0,1);
                strafeRight(.5, 0, 3);
                backward(.5, 2, 21);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                forward(.5, 0, 12);
                toAngle(0, 1);
                forward(.5, 0, 20);
                shoot(-25.0, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 1, 16);
                strafeRight(.5, 0, 3);
                shoot(-25.0, (2240*28)/60.0);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 8);//12
                setIntakePower(-.3);
                forward(.5, 0, 2);
                setIntakePower(1);
                backward(.5, 0, 4);
                setIntakePower(-.3);
                forward(.5, 0, 2);
                setIntakePower(1);
                backward(.5, 0, 4);
                forward(.5, 0, 12);//6
                //shoot(-24, (2240*28)/60.0);
                //toAngle(90, 1);
                //setIntakePower(1);
                //backward(.7, 0, 16);
                //setIntakePower(-1);
                //setIntakePower(1);
                //forward(.7, 0, 8);//17
                toAngle(0,1);
                forward(1, 0, 19);
                shoot(-22, (2150*28)/60.0);
                toAngle(0,1);
                forward(1, 1, 20);
                toAngle(135, 1);
                dropWobbleGoal();
//                strafeRight(.5, 0, 2);
//                toAngle(0, 1);
//                backward(.5, 1, 2);
                break;
        }

    }
}
