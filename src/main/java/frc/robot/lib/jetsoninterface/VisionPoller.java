package frc.robot.lib.jetsoninterface;

import frc.robot.lib.jetsoninterface.Delta;
import frc.robot.Robot;

import java.util.concurrent.TimeUnit;
import java.util.Arrays;

public class VisionPoller extends Thread {
    public static String defaultServerUrl = VisionClient.defaultServerUrl;

    private String serverUrl;
    private VisionClient client;
    private TargetInfo latestTargetInfo = null;
    private String latestFailureReason = "Not yet time-synced";
    private boolean shutdownNow = false;
    private long pollCount = 0;

    private String[] allowedErrors = { "Unable to find 2 target contours", "Could not find 2 targets",
            "Hull does not have 6 vertices", "Unable to determine target pose using solvepnp" };


    public static void setDefaultUrl(String defaultUrl){
        defaultServerUrl = defaultUrl;
    }
    
    public static String normalizeUrl(String serverUrl) {
        if (serverUrl == null || serverUrl.length() == 0) {
            serverUrl = defaultServerUrl;
        }
        return serverUrl;
    }

    private VisionPoller(String serverUrl) {
        super("VisionPoller@" + normalizeUrl(serverUrl));
        this.serverUrl = normalizeUrl(serverUrl);

        client = new VisionClient(serverUrl);

        start();
    }

    public VisionPoller() {
        this(null);
    }

    public VisionClient getClient() {
        return client;
    }

    public TargetResult getLatestTargetResult() {
        String reason;
        TargetInfo info;
        TargetResult result;
        synchronized (this) {
            reason = latestFailureReason;
            info = latestTargetInfo;
        }
        if (info == null) {
            result = new TargetResult(reason);
        } else {
            result = new TargetResult(info);
        }
        return result;
    }

    public synchronized TargetResult getNewTargetInfo(TargetInfo currentTargetInfo, long maxWaitMilliseconds) {
        TargetResult result;
        long startMilli = System.nanoTime() / 1000000;
        long endMilli = startMilli + maxWaitMilliseconds;
        while (currentTargetInfo == latestTargetInfo && !shutdownNow) {
            if (maxWaitMilliseconds == 0) {
                try {
                    wait();
                } catch (InterruptedException e) {
                }
            } else {
                long currentMilli = System.nanoTime() / 1000000;
                if (currentMilli >= endMilli) {
                    break;
                }
                long waitMilli = endMilli - currentMilli;
                try {
                    wait(waitMilli);
                } catch (InterruptedException e) {
                }
            }
        }
        if (latestTargetInfo == null) {
            result = new TargetResult(latestFailureReason);
        } else if (currentTargetInfo == latestTargetInfo) {
            result = new TargetResult("A new target frame was not captured in the time allotted");
        } else {
            result = new TargetResult(latestTargetInfo);
        }
        return result;
    }

    public void close() {
        if (!shutdownNow) {
            Robot.debugStream.println("VisionPoller: beginning shutdown");
            shutdownNow = true;
            try {
                join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void run() {
        Robot.debugStream.println("VisionPoller: synchronizing time with server");
        while (!shutdownNow) {
            try {
                client.getServerShift();
                break;
            } catch (Exception e) {
                synchronized (this) {
                    e.printStackTrace();
                    latestTargetInfo = null;
                    latestFailureReason = "Unable to sync time with vision server: " + e.getMessage();
                    notifyAll();
                }

                printError(latestFailureReason);
                if (!shutdownNow) {
                    try {
                        TimeUnit.SECONDS.sleep(5);
                    } catch (InterruptedException e2) {
                    }
                }
            }
        }

        Robot.debugStream.println("VisionPoller: time successfully sync'ed with server");

        while (!shutdownNow) {
            pollCount++;
            try {
                TargetInfo info = client.getTargetInfo();
                synchronized (this) {
                    latestFailureReason = null;
                    latestTargetInfo = info;
                }
            } catch (VisionException e) {
                synchronized (this) {
                    latestTargetInfo = null;
                    latestFailureReason = "Unable to locate target: " + e.getMessage();
                    notifyAll();
                }

                printError(latestFailureReason);
            } catch (Exception e) {
                synchronized (this) {
                    latestTargetInfo = null;
                    latestFailureReason = "Unable to query vision server for target status: " + e.getMessage();
                    notifyAll();
                }

                printError(latestFailureReason);
                if (!shutdownNow) {
                    try {
                        TimeUnit.MILLISECONDS.sleep(50);
                    } catch (InterruptedException e2) {
                    }
                }
            }
        }

        Robot.debugStream.println("VisionPoller: Polling thread shutting down");

        synchronized (this) {
            latestTargetInfo = null;
            latestFailureReason = "VisionPoller is shutting down";
            notifyAll();
        }
    }

    private void printError(String errorMessage) {
        boolean allowed = false;
        Robot.debugStream.println("Error getting relative position");
        for (int i = 0; i < allowedErrors.length; i++) {
            if (errorMessage.contains(allowedErrors[i])) {
                allowed = true;
            }
        }
        if (!allowed) {
            Robot.debugStream.println("VisionPoller: " + errorMessage);
        }
    }

    // Returns the relative postion from robot to the vision target
    public Delta getRelativePosition() {
        TargetResult result = getLatestTargetResult();
        TargetInfo info = result.info;
        if (info == null) {

            this.printError(result.failureReason);
            return null;
        } else {
            return new Delta(info.y / 39.3701, info.x / 39.3701,
                    (90.0 - info.rx) * Math.PI / 180, client.serverToLocalTimeNano(info.nanoTime));
        }
    }
}