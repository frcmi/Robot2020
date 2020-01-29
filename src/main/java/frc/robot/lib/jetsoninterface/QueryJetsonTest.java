package frc.robot.lib.jetsoninterface;

import frc.robot.Robot;

import java.util.concurrent.TimeUnit;

import static java.lang.Math.PI;
//import java.lang.Math.*;

public class QueryJetsonTest {
    private static OpencvHelper cvh = OpencvHelper.getInstance();

    public static void main(String[] args) {
        String url = "http://tegra-ubuntu.local:5800";
        if (args.length > 0) {
            url = args[0];
        }

        long nano1sec = 1000000000L;

        VisionPoller vp = new VisionPoller();

        try {
            TimeUnit.MILLISECONDS.sleep(2000);
        } catch (InterruptedException e) {
        }

        VisionClient vc = vp.getClient();

        Delta d1 = new Delta(1, 1, 0, 0);

        Robot.debugStream.println("setting testDelta");

        vc.setDeltaProperty("testDelta", d1);

        Robot.debugStream.println("getting testDelta");
        Delta d2 = vc.getDeltaProperty("testDelta");

        Robot.debugStream.println("setting testDelta2");
        vc.setDeltaProperty("testDelta2", d2);

        long startTime = System.nanoTime();
        long endTime = startTime + nano1sec * 10;

        Robot.debugStream.printf("starttime=%d, endtime=%d\n", startTime, endTime);

        while (System.nanoTime() < endTime) {
            TargetResult tr = vp.getLatestTargetResult();
            if (tr.success) {
                TargetInfo info = tr.info;
                long localTime = System.nanoTime();
                Robot.debugStream.printf("POLL JETSON: Success!\n  Local Time: %d\n", localTime);
                Robot.debugStream.printf("  Cam Time: %d\n", info.nanoTime);
                cvh.printMat(info.cvRvec, "Rvec");
                cvh.printMat(info.cvTvec, "Tvec");
            } else {
                Robot.debugStream.printf("POLL JETSON: Failure: %s\n", tr.failureReason);
            }
            try {
                TimeUnit.MILLISECONDS.sleep(50);
            } catch (InterruptedException e) {
            }

        }

        Robot.debugStream.printf("Closing Vision poller at %d\n", System.nanoTime());

        vp.close();

        Robot.debugStream.println("Finished!");
    }
}