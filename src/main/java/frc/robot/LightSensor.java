package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;

public class LightSensor {

  private AnalogInput analog;
  private AnalogTrigger trigger;

  public LightSensor(int channelNum) {
    analog = new AnalogInput(channelNum);
    trigger = new AnalogTrigger(analog);
    analog.setAverageBits(2);
    trigger.setLimitsVoltage(1, 3);
  }

  public boolean isSensed() {
    return !trigger.getTriggerState();
  }

  public double volts() {
    return analog.getVoltage();
  }
}
