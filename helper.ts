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
    
    /**
     * Stift Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=robotbit_stift_rauf block="Stift rauf|%index"
    //% group="Linea" weight=100
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
     //% subcategory="Linea" weight=100
    export function Stift_rauf(index: Servos): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = (50 * 1800 / 180 + 600) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }
    /**
     * Stift Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=robotbit_stift_runter block="Stift runter|%index"
    //% group="Linea" weight=100
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
     //% subcategory="Linea" weight=100
    export function Stift_runter(index: Servos): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = (90 * 1800 / 180 + 600) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }
    
    //% blockId=robotbit_manuva_rechts block="rechts |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function ManuvaRechts(delay: number): void {
        MotorRun(0x1, -200);
        MotorRun(0x2, -200);
        MotorRun(0x3, 200);
        MotorRun(0x4, 200);
        basic.pause(delay);
        MotorStopAll()
    }
    //% blockId=robotbit_Manuva_vor block="vorwärts |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function ManuvaVor(delay: number): void {
        MotorRun(0x1, 200);
        MotorRun(0x2, 200);
        MotorRun(0x3, 200);
        MotorRun(0x4, 200);
        basic.pause(delay);
        MotorStopAll()
    }
    //% blockId=robotbit_manuva_zur block="rückwärts |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function ManuvaZur(delay: number): void {
        MotorRun(0x1, -200);
        MotorRun(0x2, -200);
        MotorRun(0x3, -200);
        MotorRun(0x4, -200);
        basic.pause(delay);
        MotorStopAll()
    }
     //% blockId=robotbit_Manuva_links block="links |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function ManuvaLinks(delay: number): void {
        MotorRun(0x1, 200);
        MotorRun(0x2, 200);
        MotorRun(0x3, -200);
        MotorRun(0x4, -200);
        basic.pause(delay);
        MotorStopAll()
    }
      //% blockId=robotbit_Manuva_linksvordiag block="Diagonal Links Vor |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function Manuvalvd(delay: number): void {
        MotorRun(0x1, 200);
        MotorRun(0x4, 200);
        basic.pause(delay);
        MotorStopAll()
    }
     //% blockId=robotbit_Manuva_rechtsvordiag block="Diagonal Rechts Vor |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function Manuvarvd(delay: number): void {
        MotorRun(0x2, 200);
        MotorRun(0x3, 200);
        basic.pause(delay);
        MotorStopAll()
    }
     //% blockId=robotbit_Manuva_linksrueckdiag block="Diagonal Links Zurück |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function Manuvalrd(delay: number): void {
        MotorRun(0x2, -200);
        MotorRun(0x3, -200);
        basic.pause(delay);
        MotorStopAll()
    }
     //% blockId=robotbit_Manuva_rechtsrueckdiag block="Diagonal Rechts Zurück |Dauer %delay|Millisekunden"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Manuva" weight=110
    export function Manuvarrd(delay: number): void {
        MotorRun(0x1, -200);
        MotorRun(0x4, -200);
        basic.pause(delay);
        MotorStopAll()
    }
    
    
    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=robotbit_servo block="Servo|%index|degree %degree"
    //% group="Servo" weight=62
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
     //% subcategory="RoboterArm" weight=90
    export function Servo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = (degree * 1800 / 180 + 600) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }


    //% blockId=robotbit_stepper_degree block="Stepper 28BYJ-48|%index|degree %degree"
    //% group="Motor" weight=54
    //% subcategory="RoboterArm" weight=90
    export function StepperDegree(index: Steppers, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(index, degree > 0);
        degree = Math.abs(degree);
        basic.pause(10240 * degree / 360);
        MotorStopAll()
    }


    //% blockId=robotbit_stepper_turn block="Stepper 28BYJ-48|%index|turn %turn"
    //% group="Motor" weight=53
    //% subcategory="RoboterArm" weight=90
    export function StepperTurn(index: Steppers, turn: Turns): void {
        let degree = turn;
        StepperDegree(index, degree);
    }

    //% blockId=robotbit_stepper_dual block="Dual Stepper(Degree) |M1 %degree1| M2 %degree2"
    //% group="Motor" weight=52
    //% subcategory="RoboterArm" weight=90
    export function StepperDual(degree1: number, degree2: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(1, degree1 > 0);
        setStepper(2, degree2 > 0);
        degree1 = Math.abs(degree1);
        degree2 = Math.abs(degree2);
        basic.pause(10240 * Math.min(degree1, degree2) / 360);
        if (degree1 > degree2) {
            stopMotor(3); stopMotor(4);
            basic.pause(10240 * (degree1 - degree2) / 360);
        } else {
            stopMotor(1); stopMotor(2);
            basic.pause(10240 * (degree2 - degree1) / 360);
        }

        MotorStopAll()
    }

    /**
     * Stepper Car move forward
     * @param distance Distance to move in cm; eg: 10, 20
     * @param diameter diameter of wheel in mm; eg: 48
    */
    //% blockId=robotbit_stpcar_move block="Car Forward|Distance(cm) %distance|Wheel Diameter(mm) %diameter"
    //% group="Motor" weight=51
    //% subcategory="RoboterArm" weight=90
    export function StpCarMove(distance: number, diameter: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * 10 * distance / 3 / diameter; // use 3 instead of pi
        setStepper(1, delay > 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }

    /**
     * Stepper Car turn by degree
     * @param turn Degree to turn; eg: 90, 180, 360
     * @param diameter diameter of wheel in mm; eg: 48
     * @param track track width of car; eg: 125
    */
    //% blockId=robotbit_stpcar_turn block="Car Turn|Degree %turn|Wheel Diameter(mm) %diameter|Track(mm) %track"
    //% group="Motor" weight=50
    //% blockGap=50
    //% subcategory="RoboterArm" weight=90
    export function StpCarTurn(turn: number, diameter: number, track: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * turn * track / 360 / diameter;
        setStepper(1, delay < 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }

    //% blockId=robotbit_motor_run block="Motor|%index|speed %speed"
    //% group="Motor" weight=59
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Auto" weight=90
    export function MotorRun(index: Motors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }
        if (index > 4 || index <= 0)
            return
        let pp = (index - 1) * 2
        let pn = (index - 1) * 2 + 1
        if (speed >= 0) {
            setPwm(pp, 0, speed)
            setPwm(pn, 0, 0)
        } else {
            setPwm(pp, 0, 0)
            setPwm(pn, 0, -speed)
        }
    }


    /**
     * Execute two motors at the same time
     * @param motor1 First Motor; eg: M1A, M1B
     * @param speed1 [-255-255] speed of motor; eg: 150, -150
     * @param motor2 Second Motor; eg: M2A, M2B
     * @param speed2 [-255-255] speed of motor; eg: 150, -150
    */
    //% blockId=robotbit_motor_dual block="Motor|%motor1|speed %speed1|%motor2|speed %speed2"
    //% group="Motor" weight=58
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Auto" weight=90
    
    export function MotorRunDual(motor1: Motors, speed1: number, motor2: Motors, speed2: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
    }

   




    /**
     * Execute single motors with delay
     * @param index Motor Index; eg: M1A, M1B, M2A, M2B
     * @param speed [-255-255] speed of motor; eg: 150, -150
     * @param delay seconde delay to stop; eg: 1
    */
    //% blockId=robotbit_motor_rundelay block="Motor|%index|speed %speed|delay %delay|s"
    //% group="Motor" weight=57
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Auto" weight=90
    export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
        MotorRun(index, speed);
        basic.pause(delay * 1000);
        MotorRun(index, 0);
    }



    //% blockId=robotbit_stop block="Motor Stop|%index|"
    //% group="Motor" weight=56
    //% subcategory="Auto" weight=90
    export function MotorStop(index: Motors): void {
        MotorRun(index, 0);
    }

    //% blockId=robotbit_stop_all block="Motor Stop All"
    //% group="Motor" weight=55
    //% blockGap=50
    //% subcategory="Auto" weight=90
    export function MotorStopAll(): void {
        if (!initialized) {
            initPCA9685()
        }
        for (let idx = 1; idx <= 4; idx++) {
            stopMotor(idx);
        }
    }


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