package frc.robot.lib.jetsoninterface.model;

import frc.robot.lib.jetsoninterface.Delta;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class GetDeltaPropertyResponse extends Response {
    public Delta data;

    @JsonCreator
    public GetDeltaPropertyResponse(
            @JsonProperty("success") boolean success,
            @JsonProperty("error") ErrorInfo error,
            @JsonProperty("ts_request_mono") double ts_request_mono,
            @JsonProperty("data") Delta data)
    {
        super(success, error, ts_request_mono);
        this.data = data;
    }
}
