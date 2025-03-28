package frc8768.robot.subsystems.rgb;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

public class RGBIOCANdle implements RGBIO {
  private final CANdle candle;

  public RGBIOCANdle() {
    candle = new CANdle(RGBConstants.RGB_CONFIGS.id());
    candle.configLOSBehavior(true);
    candle.configLEDType(LEDStripType.GRB);
  }

  public void updateInputs(RGBIOInputs inputs) {}

  public void displayMessage(RGBConstants.RGBMessage message) {
    switch (message.getPattern()) {
      case STROBE -> candle.animate(
          new StrobeAnimation(
              message.getColor().r,
              message.getColor().g,
              message.getColor().b,
              0,
              .2,
              RGBConstants.RGB_CONFIGS.numLEDs()));
      case SOLID -> candle.animate(
          new TwinkleAnimation(
              message.getColor().r,
              message.getColor().g,
              message.getColor().b,
              0,
              0,
              RGBConstants.RGB_CONFIGS.numLEDs(),
              TwinklePercent.Percent100));
      case RAINBOW -> candle.animate(
          new RainbowAnimation(.2, .5, RGBConstants.RGB_CONFIGS.numLEDs()));
      case FIRE -> candle.animate(
          new FireAnimation(0.7, 0.2, RGBConstants.RGB_CONFIGS.numLEDs(), 1, 0.1));
      case BOUNCE -> candle.animate(
          new LarsonAnimation(
              message.getColor().r,
              message.getColor().g,
              message.getColor().b,
              0,
              .5,
              RGBConstants.RGB_CONFIGS.numLEDs(),
              LarsonAnimation.BounceMode.Front,
              7));
      case PULSE -> candle.animate(
          new SingleFadeAnimation(
              message.getColor().r,
              message.getColor().g,
              message.getColor().b,
              0,
              0.7,
              RGBConstants.RGB_CONFIGS.numLEDs()));
    }
  }

  public void clear() {}
}
