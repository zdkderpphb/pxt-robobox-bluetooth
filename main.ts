/*
Riven
modified from pxt-servo/servodriver.ts
load dependency
"robotbit": "file:../pxt-robotbit"
*/
const enum DistanceUnit {
    //% block="cm"
    CM = 58, // Duration of echo round-trip in Microseconds (uS) for two centimeters, 343 m/s at sea level and 20°C
    //% block="inch"
    INCH = 148, // Duration of echo round-trip in Microseconds (uS) for two inches, 343 m/s at sea level and 20°C
  }

//% color="#63AEE6" weight=10 icon="\uf19d"
namespace Robobox {
    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    // HT16K33 commands
    const HT16K33_ADDRESS = 0x70
    const HT16K33_BLINK_CMD = 0x80
    const HT16K33_BLINK_DISPLAYON = 0x01
    const HT16K33_BLINK_OFF = 0
    const HT16K33_BLINK_2HZ = 1
    const HT16K33_BLINK_1HZ = 2
    const HT16K33_BLINK_HALFHZ = 3
    const HT16K33_CMD_BRIGHTNESS = 0xE0

    export enum Servos {
        S1 = 0x01,
        S2 = 0x02,
        S3 = 0x03,
        S4 = 0x04,
        S5 = 0x05,
        S6 = 0x06,
        S7 = 0x07,
        S8 = 0x08
    }

    export enum Motors {
        M1A = 0x1,
        M1B = 0x2,
        M2A = 0x3,
        M2B = 0x4
    }

    export enum Steppers {
        M1 = 0x1,
        M2 = 0x2
    }

    export enum SonarVersion {
        V1 = 0x1,
        V2 = 0x2
    }

    export enum Turns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }

    let initialized = false
    let initializedMatrix = false
    let neoStrip: neopixel.Strip;
    let matBuf = pins.createBuffer(17);
    let distanceBuf = 0;

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        //serial.writeValue("ch", channel)
        //serial.writeValue("on", on)
        //serial.writeValue("off", off)

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }


    function setStepper(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(0, STP_CHA_L, STP_CHA_H);
                setPwm(2, STP_CHB_L, STP_CHB_H);
                setPwm(1, STP_CHC_L, STP_CHC_H);
                setPwm(3, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(3, STP_CHA_L, STP_CHA_H);
                setPwm(1, STP_CHB_L, STP_CHB_H);
                setPwm(2, STP_CHC_L, STP_CHC_H);
                setPwm(0, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(4, STP_CHA_L, STP_CHA_H);
                setPwm(6, STP_CHB_L, STP_CHB_H);
                setPwm(5, STP_CHC_L, STP_CHC_H);
                setPwm(7, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(7, STP_CHA_L, STP_CHA_H);
                setPwm(5, STP_CHB_L, STP_CHB_H);
                setPwm(6, STP_CHC_L, STP_CHC_H);
                setPwm(4, STP_CHD_L, STP_CHD_H);
            }
        }
    }

    function stopMotor(index: number) {
        setPwm((index - 1) * 2, 0, 0);
        setPwm((index - 1) * 2 + 1, 0, 0);
    }

    /**
     * Init RGB pixels mounted on robotbit
     */
    //% blockId="robotbit_rgb" block="RGB"
    //% weight=70
    export function rgb(): neopixel.Strip {
        if (!neoStrip) {
            neoStrip = neopixel.create(DigitalPin.P16, 4, NeoPixelMode.RGB)
        }

        return neoStrip;
    }
//% blockId=robotbit_Beebot_vor block="vorwärts |Dauer %delay|Millisekunden"
    //% group="Beebot" weight=54
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Basic" weight=90
    export function BeeVor(delay: number): void {
      MotorRun(0x1, 200);
      MotorRun(0x2, 200);
      MotorRun(0x3, 200);
      MotorRun(0x4, 200);
      basic.pause(delay);
      MotorStopAll()
  }
  //% blockId=robotbit_Beebot_zur block="rückwärts |Dauer %delay|Millisekunden"
  //% group="Beebot" weight=55
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Basic" weight=90
  export function BeeZur(delay: number): void {
      MotorRun(0x1, -200);
      MotorRun(0x2, -200);
      MotorRun(0x3, -200);
      MotorRun(0x4, -200);
      basic.pause(delay);
      MotorStopAll()
  }
   //% blockId=robotbit_Beebot_links block="links |Dauer %delay|Millisekunden"
  //% group="Beebot" weight=56
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Basic" weight=90
  export function BeeLinks(delay: number): void {
      MotorRun(0x1, 200);
      MotorRun(0x2, 200);
      MotorRun(0x3, -200);
      MotorRun(0x4, -200);
      basic.pause(delay);
      MotorStopAll()
  }
  //% blockId=robotbit_Beebot_rechts block="rechts |Dauer %delay|Millisekunden"
  //% group="Beebot" weight=57
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Basic" weight=90
  export function BeeRechts(delay: number): void {
      MotorRun(0x1, -200);
      MotorRun(0x2, -200);
      MotorRun(0x3, 200);
      MotorRun(0x4, 200);
      basic.pause(delay);
      MotorStopAll()
  }
  
    function triggerPulse() {
      // Reset trigger pin
      pins.setPull(ultrasonicState.trig, PinPullMode.PullNone);
      pins.digitalWritePin(ultrasonicState.trig, 0);
      control.waitMicros(2);
  
      // Trigger pulse
      pins.digitalWritePin(ultrasonicState.trig, 1);
      control.waitMicros(10);
      pins.digitalWritePin(ultrasonicState.trig, 0);
    }
  
    function getMedianRRT(roundTrips: UltrasonicRoundTrip[]) {
      const roundTripTimes = roundTrips.map((urt) => urt.rtt);
      return median(roundTripTimes);
    }
  
    // Returns median value of non-empty input
    function median(values: number[]) {
      values.sort((a, b) => {
        return a - b;
      });
      return values[(values.length - 1) >> 1];
    }
  
    function measureInBackground() {
      const trips = ultrasonicState.roundTrips;
      const TIME_BETWEEN_PULSE_MS = 145;
  
      while (true) {
        const now = input.runningTime();
  
        if (trips[trips.length - 1].ts < now - TIME_BETWEEN_PULSE_MS - 10) {
          ultrasonicState.roundTrips.push({
            ts: now,
            rtt: MAX_ULTRASONIC_TRAVEL_TIME,
          });
        }
  
        while (trips.length > ULTRASONIC_MEASUREMENTS) {
          trips.shift();
        }
  
        ultrasonicState.medianRoundTrip = getMedianRRT(
          ultrasonicState.roundTrips
        );
  
        for (let i = 0; i < ultrasonicState.travelTimeObservers.length; i++) {
          const threshold = ultrasonicState.travelTimeObservers[i];
          if (threshold > 0 && ultrasonicState.medianRoundTrip <= threshold) {
            control.raiseEvent(
              MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID,
              threshold
            );
            // use negative sign to indicate that we notified the event
            ultrasonicState.travelTimeObservers[i] = -threshold;
          } else if (
            threshold < 0 &&
            ultrasonicState.medianRoundTrip > -threshold
          ) {
            // object is outside the detection threshold -> re-activate observer
            ultrasonicState.travelTimeObservers[i] = -threshold;
          }
        }
  
        triggerPulse();
        basic.pause(TIME_BETWEEN_PULSE_MS);
      }
    }

}
