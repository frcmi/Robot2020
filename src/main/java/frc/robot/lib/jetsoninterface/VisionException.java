package frc.robot.lib.jetsoninterface;

import frc.robot.lib.jetsoninterface.model.ErrorInfo;

public class VisionException extends RuntimeException {
    public ErrorInfo errorInfo;

    public VisionException(ErrorInfo errorInfo) {
        super(errorInfo.message);
        this.errorInfo = errorInfo;
    }
}
