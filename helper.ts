
    //Ultrasonic
    
    const MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID = 798;
    const MAX_ULTRASONIC_TRAVEL_TIME = 300 * DistanceUnit.CM;
    const ULTRASONIC_MEASUREMENTS = 3;
  
    interface UltrasonicRoundTrip {
      ts: number;
      rtt: number;
    }
  
    interface UltrasonicDevice {
      trig: DigitalPin | undefined;
      roundTrips: UltrasonicRoundTrip[];
      medianRoundTrip: number;
      travelTimeObservers: number[];
    }
  
    let ultrasonicState: UltrasonicDevice;
  
    /**
     * Configures the ultrasonic distance sensor and measures continuously in the background.
     * @param trig pin connected to trig, eg: DigitalPin.P5
     * @param echo pin connected to echo, eg: DigitalPin.P8
     */
    //% subcategory="Ultrasonic"
    //% blockId="makerbit_ultrasonic_connect"
    //% block="connect ultrasonic distance sensor | with Trig at %trig | and Echo at %echo"
    //% trig.fieldEditor="gridpicker"
    //% trig.fieldOptions.columns=4
    //% trig.fieldOptions.tooltips="false"
    //% echo.fieldEditor="gridpicker"
    //% echo.fieldOptions.columns=4
    //% echo.fieldOptions.tooltips="false"
    //% weight=80
    export function connectUltrasonicDistanceSensor(
      trig: DigitalPin,
      echo: DigitalPin
    ): void {
      if (ultrasonicState && ultrasonicState.trig) {
        return;
      }
  
      if (!ultrasonicState) {
        ultrasonicState = {
          trig: trig,
          roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
          medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
          travelTimeObservers: [],
        };
      } else {
        ultrasonicState.trig = trig;
      }
  
      pins.onPulsed(echo, PulseValue.High, () => {
        if (
          pins.pulseDuration() < MAX_ULTRASONIC_TRAVEL_TIME &&
          ultrasonicState.roundTrips.length <= ULTRASONIC_MEASUREMENTS
        ) {
          ultrasonicState.roundTrips.push({
            ts: input.runningTime(),
            rtt: pins.pulseDuration(),
          });
        }
      });
  
      control.inBackground(measureInBackground);
    }
  
    /**
     * Do something when an object is detected the first time within a specified range.
     * @param distance distance to object, eg: 20
     * @param unit unit of distance, eg: DistanceUnit.CM
     * @param handler body code to run when the event is raised
     */
    //% subcategory="Ultrasonic"
    //% blockId=makerbit_ultrasonic_on_object_detected
    //% block="on object detected once within | %distance | %unit"
    //% weight=69
    export function onUltrasonicObjectDetected(
      distance: number,
      unit: DistanceUnit,
      handler: () => void
    ) {
      if (distance <= 0) {
        return;
      }
  
      if (!ultrasonicState) {
        ultrasonicState = {
          trig: undefined,
          roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
          medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
          travelTimeObservers: [],
        };
      }
  
      const travelTimeThreshold = Math.imul(distance, unit);
  
      ultrasonicState.travelTimeObservers.push(travelTimeThreshold);
  
      control.onEvent(
        MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID,
        travelTimeThreshold,
        () => {
          handler();
        }
      );
    }
  
    /**
     * Returns the distance to an object in a range from 1 to 300 centimeters or up to 118 inch.
     * The maximum value is returned to indicate when no object was detected.
     * -1 is returned when the device is not connected.
     * @param unit unit of distance, eg: DistanceUnit.CM
     */
    //% subcategory="Ultrasonic"
    //% blockId="makerbit_ultrasonic_distance"
    //% block="ultrasonic distance in %unit"
    //% weight=60
    export function getUltrasonicDistance(unit: DistanceUnit): number {
      if (!ultrasonicState) {
        return -1;
      }
      basic.pause(0); // yield to allow background processing when called in a tight loop
      return Math.idiv(ultrasonicState.medianRoundTrip, unit);
    }
  
    /**
     * Returns `true` if an object is within the specified distance. `false` otherwise.
     *
     * @param distance distance to object, eg: 20
     * @param unit unit of distance, eg: DistanceUnit.CM
     */
    //% subcategory="Ultrasonic"
    //% blockId="makerbit_ultrasonic_less_than"
    //% block="ultrasonic distance is less than | %distance | %unit"
    //% weight=50
    export function isUltrasonicDistanceLessThan(
      distance: number,
      unit: DistanceUnit
    ): boolean {
      if (!ultrasonicState) {
        return false;
      }
      basic.pause(0); // yield to allow background processing when called in a tight loop
      return Math.idiv(ultrasonicState.medianRoundTrip, unit) < distance;
    }