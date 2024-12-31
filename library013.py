from hub import light_matrix,button,motion_sensor,port,light_matrix
import hub
import motor,color_sensor,app,color,distance_sensor
import runloop


import math
import time
import gc

# date 10/30/24

class MyTimer(object):
    def __init__(self):
        self.started = time.ticks_ms()


    def reset(self):
        self.started = time.ticks_ms()


    def getTime(self):
        x= time.ticks_ms()
        y=time.ticks_diff(x, self.started)
        return(y)

class PID(object):
    def __init__(self, KP, KI, KD, max_integral=100.0):
        self._KP = KP
        self._KI = KI
        self._KD = KD
        self.timer = MyTimer()

        self._error = 0.0
        self._integral = 0.0
        self._max_integral = max_integral if KI == 0.0 else max_integral / KI
        self._start_time = self.timer.getTime()
        self._prev_error = None
        self._prev_response = None

    def getCorrection(self, set_point: float, state: float, dt: float = 0.02, debug=False):
        """
        :param set_point:
        :param state:
        :param dt: time since last interaction in seconds
        :return:
        """
        current_time = self.timer.getTime()
        ellapsed = current_time - self._start_time
        dtReal = ellapsed
        error = set_point - state
        self._error = error
        if self._prev_error is None:
            self._prev_error = error
        self._integral += error * dtReal / (dt*1000)
        if self._max_integral and (abs(self._integral) > self._max_integral):
            self._integral = self._max_integral if self._integral > 0 else -self._max_integral
        d_error = (error - self._prev_error)
        self._prev_error = error
        response = self._KP * error * dtReal / (dt*1000) + self._KI * self._integral + self._KD * d_error * dtReal / (dt*1000)
        self._prev_response = response
        self._start_time = current_time
        if debug:
            print('PID:', self._KP, self._KI, self._KD, 'dtReal:', dtReal)
            print('set_point=', set_point, 'state=', state, 'error=', error, 'response=', response)
        return response

class MyMotor(object):
    # motor = None
    port=None
    name = ''
    isInverted = False
    invertedSign = 1
    markDegrees = 0
    startTime = 0
    lastTime = 0
    error = 0.0
    actionStart = None
    stackTimer = None
    actionDone = False
    degreesToMM = 3.14159265359 * 88 / 360
    step = 0

    def __init__(self, sensorPort='A', isInverted=False, name='motor', degreesToMM=3.14159265359 * 56 / 360,speedPowerEq=(1,0),noInit=False):
        if noInit:
            return None
        print('Motor ', port, name, 'initialized')
        self.degrees = 0
        self.speedPowerEq=speedPowerEq

        # self.motor = Motor(port)
        ports={'A':0,'B':1,'C':2,'D':3,'E':4,'F':5}
        self.port=ports[sensorPort.upper()]
        self.isInverted = isInverted
        self.name = name
        self.degreesToMM = degreesToMM
        if self.isInverted:
            self.invertedSign = -1
        else:
            self.invertedSign = 1

        try:
            self.markDegrees = motor.relative_position(self.port)
        except:
            light_matrix.write(sensorPort)
            time.sleep(2)
            raise SystemError()
        self.timer=MyTimer()
        self.startTime = 0
        self.lastTime = 0

    def zero(self, offset=0):
        """Move the motor to the zero position.

        Args:
            offset: The offset to add to the zero position
        """
        if offset < 0:
            angle = 360 + offset
        else:
            angle = offset

        motor.run_to_absolute_position(self.port,0,200,direction= motor.SHORTEST_PATH,stop=motor.HOLD)
        while True:
            a=motor.absolute_position(self.port)
            if abs(a)<3:
                break

        self.resetDegrees()



    def resetDegrees(self):
        """Reset the inner counter of degrees of the moter
        """
        self.markDegrees = motor.relative_position(self.port)


    def getDegrees(self):
        """Return the degrees of the motor.
        """
        return ((motor.relative_position(self.port) - self.markDegrees) * self.invertedSign)

    def setDegrees(self, degrees=30, speed=20, acceptedError=3, stallDetectionTime=1,hold=False,debug=False):
        """Turn Motor to specific degrees

        Args:
            degrees (int, optional): the degrees that motor need to move to. Defaults to 30.
            speed (int, optional): the speed of the motor. Defaults to 20.
            acceptedError (int, optional): +/- accepted error. Defaults to 3.
            stallDetectionTime (float, optional): time that mottor stays without move to detect stack. Defaults to 0.5.
            debug (bool, optional): if True prints some information. Defaults to False.
        """
        currentAngle = self.getDegrees()
        if abs(currentAngle - degrees) <= acceptedError:
            self.stop()
            self.step = 0
            self.actionDone = True
            if debug:
                print('setDegrees Completed ok')
                print('Target Degrees ', degrees, 'final degrees ', currentAngle)
            return (1)
        if degrees > currentAngle:
            self.speed = speed
        else:
            self.speed = -speed
        if self.step == 0:
            self.actionStart = self.timer.getTime() / 1000
            self.actionDone = False
            self.stackTimer = None
            self.step += 1
            if debug:
                print('current degrees ', currentAngle, 'target degrees ', degrees)
        else:
            self.step += 1
            if stallDetectionTime > 0:

                if self.getSpeed() == 0:
                    if self.stackTimer == None:
                        self.stackTimer = self.timer.getTime() / 1000
                    else:
                        t1 = self.timer.getTime() / 1000
                        dt1 = t1 - self.stackTimer
                        if dt1 > stallDetectionTime:
                            self.stop(hold=hold)
                            self.step = 0
                            self.actionDone = True
                            if debug:
                                print('Stall detected')
                                print('Target Degrees ', degrees, 'final degrees ', currentAngle,'stalled time',dt1)
                            return (1)
                else:
                    self.stackTimer = None

        self.setSpeed(speed=self.speed)
        return (0)

    def goToAngle(self,angle=-30,speed=20,acceptedError=3,stallDetectionTime=1,hold=False,debug=False):
        """Move the motor to specific angle.

        Args:
            angle (int,optional): the angle that motor need to move to. Defaults to -30.
            speed (int,optional): the speed of the motor. (always possitive) Defaults to 20.
            acceptedError (int,optional): +/- accepted error. Defaults to 3.
            stallDetectionTime (float,optional): time that mottor stays without move to detect stack. Defaults to 0.5
            hold (bool,optional): if True motor will not stop when reaching the target. Defaults to False.
            debug (bool,optional): if True prints some information. Defaults to False."""
        target = angle
        speed1=speed
        self.actionDone = False
        while not self.actionDone:
            self.setDegrees(degrees=target, speed=speed1, debug=debug,stallDetectionTime=stallDetectionTime,hold=hold)

            if abs(target - self.getDegrees()) <= acceptedError:
                self.stop()
                success = True
                break


    def getDistance(self):
        """Returns the Distance that covered the motor (degreesToMM must setted correctly)
        """
        return (self.getDegrees() * self.degreesToMM)

    def getSpeed(self):
        """Returns the current speed of the motor
        """
        return (motor.velocity(self.port)/1 * self.invertedSign)


    def getDt(self):
        """Returns the time that elapsed since last action"""
        return ((self.timer.getTime() - self.lastTime) / 1000)

    def getActionTime(self):
        """Returns the time that elapsed since last action"""
        return (self.timer.getTime() / 1000)

    def on(self, power=20, debug=False):
        """Turn the motor on

        Args:
            power (int): the power of the motor. Defaults to 20.
            debug (bool, optional): if True prints some information. Defaults to False."""
        motor.run(self.port,power*10*self.invertedSign)

    def run(self, speed=20, debug=False):
        """Run the motor as a specific speed

        Args:
            speed (int): the speed of the motor. (always possitive) Defaults to 20.
            debug (bool,optional): if True prints some information. Defaults to False."""
        power=(self.speedPowerEq[0]* speed + self.speedPowerEq[1])
        velocity=int(power*10)
        motor.run(self.port,velocity*self.invertedSign)


    def setSpeed(self, speed=20, debug=False, kp=0.5):
        """Set the speed of the motor. Use this function inside a loop to control the speed of the motor.

        Args:
            speed (int): the speed +/- of the motor. Defaults to 20.
            debug (bool,optional): if True prints some information. Defaults to False.
            kp (float,optional): the proportional gain. Defaults to 0.5."""
        if not self.startTime:
            self.startTime = self.timer.getTime()
            self.lastTime = self.timer.getTime()
            self.error = 0
            self.sumError = 0
        if debug:
            self.error += (self.getSpeed() - speed) * (self.timer.getTime() - self.lastTime) / 1000

        currentSpeed = self.getSpeed()
        error = (speed - currentSpeed)
        self.sumError += error
        sumCorrection = self.sumError * 0.01
        if sumCorrection > 20:
            sumCorrection = 20
        elif sumCorrection < -20:
            sumCorrection = -20
        correction = error * kp
        if abs(currentSpeed) < abs(speed) * 0.7:
            speed1 = (currentSpeed + speed) / 2 + sumCorrection
        else:
            speed1 = (speed + correction + sumCorrection)

        if speed1 > 100:
            speed1 = 100
        if speed1 < -100:
            speed1 = -100
        # speed1=speed

        if speed1 > 0:
            if speed1 > int(speed1):
                speed2 = int(speed1) + 1
            elif speed1 < int(speed1):
                speed2 = int(speed1) - 1
            else:
                speed2 = int(speed1)
        elif speed1 < 0:
            if speed1 < int(speed1):
                speed2 = int(speed) - 1
            elif speed1 > int(speed1):
                speed2 = int(speed1) + 1
            else:
                speed2 = int(speed1)
        else:
            speed2 = int(speed1)

        if currentSpeed != speed2:
            self.run(speed=speed2)

        self.lastTime = self.timer.getTime()

    def stop(self,hold=False):
        """Stop the motor

        Args:
            hold (bool,optional): if True motor will stay when reaches the target. Defaults to False."""
        if hold:
            motor.stop(self.port,stop=motor.HOLD)

        else:
            motor.stop(self.port)
        self.startTime = 0
        self.lastTime = 0
        self.step=0

    def stats(self,df, i):
        # Compute the average of a column of data
        # df is a list of lists of floats, i is the index of the data within
        # df to average.
        ave = sum([l[i] for l in df]) / len(df)
        return ave

    def interpolate(self,df):
        # df is a list of list of float values
        # m is the slope of the line that best fits df, b is the y-intercept
        ave_x = self.stats(df, 0)
        ave_y = self.stats(df, 1)
        m = sum([l[0] * (l[1] - ave_y) for l in df]) / sum([l[0] * (l[0] - ave_x) for l in df])
        b = ave_y - m * ave_x

        return (m, b)


    def takeData(self,minSpeed=10,maxSpeed=84,step=7):
        l=[]
        for power in range(minSpeed,maxSpeed+1,step):
            t0=self.timer.getTime()
            while True:
                self.on(power=power)
                t1=self.timer.getTime()
                if t1-t0>1000:
                    x=self.getSpeed()
                    l.append((x,power))
                    break
        for power in range(maxSpeed,-maxSpeed-1,-step):
            t0=self.timer.getTime()
            while True:
                self.on(power=power)
                t1=self.timer.getTime()
                if t1-t0>1000:
                    x=self.getSpeed()
                    l.append((x,power))
                    break
        for power in range(-maxSpeed,step,step):
            t0=self.timer.getTime()
            while True:
                self.on(power=power)
                t1=self.timer.getTime()
                if t1-t0>1000:
                    x=self.getSpeed()
                    l.append((x,power))
                    break
        self.stop()
        # return(l)
        y=self.interpolate(l)
        print('power=a * speed + b',' (a,b)=(',y[0],',',y[1],')')
        return(y)

    def test(self, speed=20):
        i = 0
        while True:
            self.setSpeed(speed=speed, debug=True)
            i += 1
            num = 3000
            if i >= num:
                print('dt: ', self.getActionTime() / num)
                print('error: ', self.error)
                self.stop()
                break

    def printStatus(self):
        """Print informations about the motor"""
        print('=======', self.name, '=======')
        print('| isInverted:', self.isInverted)
        print('| invertedSign:', self.invertedSign)
        print('| markDegrees:', self.markDegrees)
        print('| degrees:', self.getDegrees())
        print('| distance:', self.getDistance())
        print('| isInverted:', self.isInverted)
        print('| startTime:', self.startTime)
        print('| error:', self.error)
        print('===========================')

class MyColorSensor(object):
    # colorSensor = None
    port=None
    name = 'colorSensor'
    # colorNames = ['no color', 'black', 'blue', 'green', 'yellow', 'red', 'white', 'brown']
    colorNames={
        color.UNKNOWN:'UNKNOWN'
        ,color.TURQUOISE:'TURQUOISE'
        ,color.AZURE:'AZURE'
        ,color.BLACK:'BLACK'
        ,color.BLUE:'BLUE'
        ,color.GREEN:'GREEN'
        ,color.MAGENTA:'MAGENTA'
        ,color.ORANGE:'ORANGE'
        ,color.PURPLE:'PURPLE'
        ,color.RED:'RED'
        ,color.WHITE:'WHITE'
        ,color.YELLOW:'YELLOW'
    }

    # στα επόμενα ευρετήρια καταγράφονται μέγιστες και ελάχιστες τιμές για
    # αντανάκλαση (white) και red, green, blue του αισθητήρα χρώμματος.
    maxs = {'white': 40, 'red': 0, 'green': 0, 'blue': 0}
    mins = {'white': 100, 'red': 255, 'green': 255, 'blue': 255}
    # μεταβλητή που χρησιμοποιείτε για συγχρονισμό των threads που συμμετέχει ο αισθητήρας
    actionDone = False
    # Όταν η μεταβλητή smartAdjust είναι αληθής, γίνεται αναγωγή των τιμών R,G,B,
    # στην κλίμακα 0-100 (βάσει των τιμών min,max των RGB)
    smartAdjust = False
    # Η τελευταία ανοιγμένη τιμή της αντανάκλασης (στην κλίμακα 0-100)
    value = 0
    # mode0: reflected light1:rgb
    mode = 1
    # Η τελευταία πραγματική τιμή της αντανάκλασης (όπως διαβάστηκε απο τον αισθητήρα)
    originalValue = 0
    # last original value (to obtain light changing)
    lastValue = None
    # η λίστα με τις τελευταίες τιμές αντανάκλασης που διαβάστηκαν απο τον αισθητήρα χρώμματος
    values = []
    # στοίβα με αντανακλάσεις (ώθηση μέσω μεθόδου Push)
    lightStack = []
    # η λίστα με τις τελευταίες τιμές RGB και Color που διαβάστηκαν και αναγνωρίστηκαν
    # απο τον αισθητήρα χρώμματος
    RGBCvalues = []
    colors = []
    colorsRGB = {2: [0, 0, 0], 3: [0, 0, 0], 4: [0, 0, 0], 5: [0, 0, 0]}

    # minValue = 100
    # maxValue = 0
    # η τιμή όριο μεταξύ άσπρου και μαύρου (πραγματική και όχι ανοιγμένη τιμή).
    medValue = 50
    # minRGBValue = 0
    # maxRGBValue = 10
    # medRGBValue = 5

    # οι τελευταίες τιμές των αντανακλάσεων RGB
    red = 0
    green = 0
    blue = 0
    # η τελευταία τιμή RGB που διαβάστηκε απο τον αισθητήρα
    RGB = []
    intensity=0
    # η τελευταία τιμή RGB (χωρίς αναγωγή) που διαβάστηκε απο τον αισθητήρα
    originalRGB = []
    whiteRGB=[0,0,0,0]
    blackRGB=[100,100,100,100]

    # Το τελευταίο Χρώμα που διαβάστηκε απο τον αισθητήρα
    color = 0
    lastColor=-1
    # στοίβα με χρώμματα (ώθηση μέσω μεθόδου Push)
    colorStack = []
    # χρησιμοποιείτε όταν υπάρχουν πολλαπλές αναγνώσεις RGB για την παραγωγή
    # μιας τελικής τριάδας RGB
    RGB_Read_Set = []

    def __init__(self, sensorPort='A', mins={'white': 40, 'red': 255, 'green': 255, 'blue': 255},
                maxs={'white': 90, 'red': 0, 'green': 0, 'blue': 0}, name='color1',noInit=False):
        # self.colorSensor = ColorSensor(port)
        ports={'A':0,'B':1,'C':2,'D':3,'E':4,'F':5}
        if noInit:
            return None
        self.name = name
        self.mode = 1
        self.color = 0
        self.port=ports[sensorPort.upper()]

        # self.port = port
        self.maxs = maxs.copy()
        self.mins = mins.copy()
        self.originalValue = 0
        self.lastValue = None
        self.values = []
        self.RGBCvalues = []
        self.colorStack = []
        self.lightStack = []
        whiteRGB=[0,0,0,0]
        blackRGB=[100,100,100,100]
        try:
            color_sensor.rgbi(self.port)
        except:
            light_matrix.write(sensorPort)
            time.sleep(2)
            raise SystemError()



        for i in range(9):
            self.getLight()
        if maxs['red'] + maxs['green'] + maxs['blue'] == 0:
            self.smartAdjust = False
        else:
            self.smartAdjust = True
        self.timer=MyTimer()

    def getLight(self, adjusted=True, logValues=True):
        """Get the reflected light value from the sensor

        Args:
            adjusted (bool, optional): if True the light value is adjusted to the range 0-100. Defaults to True.
            logValues (bool, optional): if True the light value is logged. Defaults to True.
            """
        # print('getlight port',self.port)
        try:
            self.getRGB(adjusted=adjusted)
            lx=self.value
        except:
            lx=48
            print('error in getLight in port:',self.port)
        if self.lastValue != None:
            self.lastValue = self.originalValue
        else:
            self.lastValue = lx
        self.originalValue = lx

        self.medValue = (self.maxs['white'] + self.mins['white']) / 2.0

        if logValues:
            self.values.append(lx)
            if len(self.values) > 9:
                self.values = self.values[1:]
        return (lx)

    def getRed(self, adjusted=True):
        """Returns the red value of the sensor

        Args:
            adjusted (bool, optional): if True the color value is adjusted to the range 0-100. Defaults to True.
        """
        self.originalValue = rgb = color_sensor.rgbi(self.port)[0] / 10.24
        color = 'red'
        if (self.originalValue > self.maxs[color]):
            self.maxs[color] = self.originalValue
        if (self.originalValue < self.mins[color]):
            self.mins[color] = self.originalValue
        self.medValue = (self.maxs[color] + self.mins[color]) / 2.0

        if self.mins[color] == self.maxs[color]:
            self.value = 48
            # self.value=self.originalValue
        else:
            self.red = (self.originalValue - self.mins[color]) * 100.0 / (self.maxs[color] - self.mins[color])
        if adjusted:
            return (self.red)
        else:
            return (self.originalValue)

    def getGreen(self, adjusted=True):
        """Returns the green value of the sensor

        Args:
            adjusted (bool, optional): if True the color value is adjusted to the range 0-100. Defaults to True.
        """
        self.originalValue = color_sensor.rgbi(self.port)[1] / 10.24
        color = 'green'
        if (self.originalValue > self.maxs[color]):
            self.maxs[color] = self.originalValue
        if (self.originalValue < self.mins[color]):
            self.mins[color] = self.originalValue
        self.medValue = (self.maxs[color] + self.mins[color]) / 2.0

        if self.mins[color] == self.maxs[color]:
            self.value = 48
            # self.value=self.originalValue
        else:
            self.green = (self.originalValue - self.mins[color]) * 100.0 / (self.maxs[color] - self.mins[color])
        if adjusted:
            return (self.green)
        else:
            return (self.originalValue)

    def getBlue(self, adjusted=True):
        """:Returns the blue value of the sensor

        Args:
            adjusted (bool, optional): if True the color value is adjusted to the range 0-100. Defaults to True.
        """
        self.originalValue = color_sensor.rgbi(self.port)[2] / 10.24
        color = 'blue'
        if (self.originalValue > self.maxs[color]):
            self.maxs[color] = self.originalValue
        if (self.originalValue < self.mins[color]):
            self.mins[color] = self.originalValue
        self.medValue = (self.maxs[color] + self.mins[color]) / 2.0

        if self.mins[color] == self.maxs[color]:
            self.value = 48
            # self.value=self.originalValue
        else:
            self.blue = (self.originalValue - self.mins[color]) * 100.0 / (self.maxs[color] - self.mins[color])
        if adjusted:
            return (self.blue)
        else:
            return (self.originalValue)

    def getIntensity(self):
        """Returns the intensity of the sensor
        """
        rgb = color_sensor.rgbi(self.port)# self.colorSensor.get_rgb_intensity()
        self.intensity=rgb[3]
        return(rgb[3])


    def getRGB(self, adjusted=True):
        """Returns the RGB value of the sensor

        Args:
            adjusted (bool, optional): if True the RGB values are adjusted to the range 0-100. Defaults to True.
        """
        rgb = color_sensor.rgbi(self.port)# self.colorSensor.get_rgb_intensity()
        self.originalRGB=rgb
        if sum(rgb)>sum(self.whiteRGB):
            self.whiteRGB=rgb
        if sum(rgb)<sum(self.blackRGB):
            self.blackRGB=rgb

        self.intensity=rgb[3]/10.24
        self.originalValue=sum(rgb)/4/10.24

        if sum(rgb)/4 / 10.24 > self.maxs['white']:
            self.maxs['white'] = sum(rgb) /4 / 10.24

        if sum(rgb)/4 / 10.24 < self.mins['white']:
            self.mins['white'] = sum(rgb) /4 / 10.24


        if rgb[0] / 10.24 > self.maxs['red']:
            self.maxs['red'] = rgb[0] / 10.24
        if rgb[0] / 10.24 < self.mins['red']:
            self.mins['red'] = rgb[0] / 10.24

        if rgb[1] / 10.24 > self.maxs['green']:
            self.maxs['green'] = rgb[1] / 10.24
        if rgb[1] / 10.24 < self.mins['green']:
            self.mins['green'] = rgb[1] / 10.24

        if rgb[2] / 10.24 > self.maxs['blue']:
            self.maxs['blue'] = rgb[2] / 10.24
        if rgb[2] / 10.24 < self.mins['blue']:
            self.mins['blue'] = rgb[2] / 10.24

        if self.mins['white'] == self.maxs['white']:
            self.value = 48
        else:
            self.value = (sum(rgb) /4 / 10.24 - self.mins['white']) * 100.0 / (self.maxs['white'] - self.mins['white'])

        if self.mins['red'] == self.maxs['red']:
            self.red = 48
            # self.value=self.originalValue
        else:
            self.red = (rgb[0] / 10.24 - self.mins['red']) * 100.0 / (self.maxs['red'] - self.mins['red'])

        if self.mins['green'] == self.maxs['green']:
            self.green = 48
            # self.value=self.originalValue
        else:
            self.green = (rgb[1] / 10.24 - self.mins['green']) * 100.0 / (self.maxs['green'] - self.mins['green'])

        if self.mins['blue'] == self.maxs['blue']:
            self.blue = 48
            # self.value=self.originalValue
        else:
            self.blue = (rgb[2] / 10.24 - self.mins['blue']) * 100.0 / (self.maxs['blue'] - self.mins['blue'])


        self.RGB= [self.red, self.green, self.blue,self.value]
        if adjusted:
            # return (self.red, self.green, self.blue,self.intensity)
            return (self.red, self.green, self.blue,self.value)
        else:
            # return (rgb)
            return (rgb)

    def setRGB(self, rgbToSet=[900,900,900,900],adjusted=True):
        rgb = rgbToSet# self.colorSensor.get_rgb_intensity()
        self.originalRGB=rgb
        if sum(rgb)>sum(self.whiteRGB):
            self.whiteRGB=rgb
        if sum(rgb)<sum(self.blackRGB):
            self.blackRGB=rgb

        self.intensity=rgb[3]/10.24
        self.originalValue=sum(rgb)/4/10.24

        if sum(rgb)/4 / 10.24 > self.maxs['white']:
            self.maxs['white'] = sum(rgb) /4 / 10.24

        if sum(rgb)/4 / 10.24 < self.mins['white']:
            self.mins['white'] = sum(rgb) /4 / 10.24


        if rgb[0] / 10.24 > self.maxs['red']:
            self.maxs['red'] = rgb[0] / 10.24
        if rgb[0] / 10.24 < self.mins['red']:
            self.mins['red'] = rgb[0] / 10.24

        if rgb[1] / 10.24 > self.maxs['green']:
            self.maxs['green'] = rgb[1] / 10.24
        if rgb[1] / 10.24 < self.mins['green']:
            self.mins['green'] = rgb[1] / 10.24

        if rgb[2] / 10.24 > self.maxs['blue']:
            self.maxs['blue'] = rgb[2] / 10.24
        if rgb[2] / 10.24 < self.mins['blue']:
            self.mins['blue'] = rgb[2] / 10.24

        if self.mins['white'] == self.maxs['white']:
            self.value = 48
        else:
            self.value = (sum(rgb) /4 / 10.24 - self.mins['white']) * 100.0 / (self.maxs['white'] - self.mins['white'])

        if self.mins['red'] == self.maxs['red']:
            self.red = 48
            # self.value=self.originalValue
        else:
            self.red = (rgb[0] / 10.24 - self.mins['red']) * 100.0 / (self.maxs['red'] - self.mins['red'])

        if self.mins['green'] == self.maxs['green']:
            self.green = 48
            # self.value=self.originalValue
        else:
            self.green = (rgb[1] / 10.24 - self.mins['green']) * 100.0 / (self.maxs['green'] - self.mins['green'])

        if self.mins['blue'] == self.maxs['blue']:
            self.blue = 48
            # self.value=self.originalValue
        else:
            self.blue = (rgb[2] / 10.24 - self.mins['blue']) * 100.0 / (self.maxs['blue'] - self.mins['blue'])


        self.RGB= [self.red, self.green, self.blue,self.value]
        if adjusted:
            # return (self.red, self.green, self.blue,self.intensity)
            return (self.red, self.green, self.blue,self.value)
        else:
            # return (rgb)
            return (rgb)


    def getColor(self):
        """Returns the color detected by the sensor
        """
        self.lastColor=self.color
        self.color=color_sensor.color(self.port) #self.colorSensor.get_color()
        return(self.color)

    def getColorName(self):
        """Returns the name of the color detected by the sensor
        """
        val=color_sensor.color(self.port)#self.colorSensor.get_color()
        y=self.colorNames[val]
        # y=''
        # if val is color.RED:
        #    y='red'
        # elif val is color.BLUE:
        #    y='blue'
        # elif val is color.YELLOW:
        #    y='yellow'
        # elif val is color.BLACK:
        #    y='black'
        # elif val is color.GREEN:
        #    y='green'
        # elif val is color.WHITE:
        #    y='white'
        # else :
        #    y='none'
        return(y)


    def isBlack(self, threshold=40, refreshInput=False):
        """Returns True if the color detected by the sensor is black

        Args:
            threshold (int, optional): the threshold value for the Black. Defaults to 40.
            refreshInput (bool, optional): if True the sensor reads a new value, otherwise use the last readed value. Defaults to False.
        """
        if refreshInput:
            self.getLight()
        return(self.vectorSimilarity( self.whiteRGB,self.originalRGB,debug=False)<threshold/100)

    def isWhite(self, threshold=98, refreshInput=False):
        """Returns True if the color detected by the sensor is white

        Args:
            threshold (int, optional): the threshold value for the White. Defaults to 98.
            refreshInput (bool, optional): if True the sensor reads a new value, otherwise use the last readed value.
        """
        if refreshInput:
            self.getLight()
        # return self.value > threshold
        # return (self.color==10)
        return(self.vectorSimilarity( self.whiteRGB,self.originalRGB,debug=False)>threshold/100)

    def getLightChange(self, refreshInput=False):
        sum = 0
        val = 0
        if refreshInput:
            self.getLight()
        for i in range(len(self.values)):
            if i < len(self.values) // 2:
                sum -= self.values[i] * 0.01
            elif i > len(self.values) // 2:
                sum += self.values[i] * 0.01
        if len(self.values) == 9:
            val = 100 * sum / (len(self.values) // 2)
        return val

    def vectorSimilarity(self,a,b,debug=False):
        # normalize
        sum1=0
        for item in a:
            sum1+=item*item


        sum=0
        for i in range(len(a)):
            sum+=a[i]*b[i]
        if debug:
            print(sum,sum1)
        if sum1>sum:
            return(sum/sum1)
        elif sum>sum1:
            return(sum1/sum)
        else:
            return(1)



    def test(self):
        # getLight test
        i = 0
        start = self.timer.getTime()
        while True:
            self.getLight()
            i += 1
            num = 3000
            if i >= num:
                end = self.timer.getTime()
                print('getLight dt: ', (end - start) / num / 3000)
                break
        # getColor test
        i = 0
        start = self.timer.getTime()
        while True:
            self.getColor()
            i += 1
            num = 3000
            if i >= num:
                end = self.timer.getTime()
                print('getColor dt: ', (end - start) / num / 3000)
                break

    def printStatus(self):
        """Prints informations about the sensor"""
        print('=======', self.name, '=======')
        print('| init string follows')
        print("color1=MyColorSensor(port='" + str(self.port) + "',mins=", self.mins, ",maxs=", self.maxs, ")")
        print('| mins:', self.mins)
        print('| maxs:', self.maxs)
        print('| values:', self.values)
        print('===========================')

class MyUltraSonic(object):
    ultraSonic = None

    def __init__(self, port='A',name='ultraSonic',noInit=False):
        if noInit:
            return None
        ports={'A':0,'B':1,'C':2,'D':3,'E':4,'F':5}
        self.port=ports[port.upper()]
        print('Sensor',name, 'initialized','(port:',port,')')

    def getDistance(self):
        """Returns the distance detected by the sensor"""
        y=distance_sensor.distance(self.port)
        return(y)

class MyForceSensor(object):
    forceButton = None

    def __init__(self, port='A',name='force',noInit=False):
        if noInit:
            return None
        ports={'A':0,'B':1,'C':2,'D':3,'E':4,'F':5}
        self.port=ports[port.upper()]
        print('Sensor',name, 'initialized','(port:',port,')')

    def getDistance(self):
        """Returns the distance of by the sensor"""
        dist = self.forceButton.distance()
        return(dist)

    def getForce(self):
        """Returns the force detected by the sensor"""
        force = self.forceButton.force()
        return(force)

    def getValues(self):
        """Prints and Returns the values of the sensor"""
        force = self.forceButton.force()
        dist = self.forceButton.distance()
        press = self.forceButton.pressed()
        touch = self.forceButton.touched()
        print('force:',force)
        print('distance:',dist)
        print('press:',press)
        print('touch:',touch)
        return(force,dist,press,touch)

    def isPressed(self):
        """Returns True if the button is pressed"""
        press = self.forceButton.pressed()
        return press

    def isTouched(self):
        """Returns True if the button is touched"""
        touch = self.forceButton.touched()
        return touch

class MyRobot(object):
    # hub = None
    # motion_sensor = hub.motion_sensor()
    leftMotor = MyMotor(noInit=True)
    rightMotor = MyMotor(noInit=True)
    motor1 = MyMotor(noInit=True)
    motor2 = MyMotor(noInit=True)
    color1 = MyColorSensor(noInit=True)
    color2 = MyColorSensor(noInit=True)
    color3 = MyColorSensor(noInit=True)
    color4 = MyColorSensor(noInit=True)
    angle = 0
    speed = 0
    lastAngle = 0
    zeroAngle = 0
    finalAngle = 0
    angleList=[]
    angleData={}
    angleError = 0
    angleErrorList = []
    angleErrorRate = 0.0
    step = 0
    step3 = 0
    actionStart = None
    sum1 = 0
    sum2 = 0
    foundLine = False
    wheelToWheelDistance=130
    sensorToSensorDistance=80
    sensorToWheelDistance=40
    distanceBetweenColorSensors=110
    debug=False

    def __init__(self,wheelToWheelDistance=130,sensorToSensorDistance=80,sensorToWheelDistance=40):
        self.wheelToWheelDistance=wheelToWheelDistance
        self.sensorToSensorDistance=sensorToSensorDistance
        self.sensorToWheelDistance=sensorToWheelDistance
        self.lastDistance=0
        self.lastScanDistance = None
        self.scanDistance = None
        self.scanDistances = None
        self.scanColor = None
        self.scanColorList = None
        self.power = None
        self.pid2 = None
        self.startingAngle = 0
        self.pid = None
        self.sumError = 0
        self.angleError = 0
        self.angleErrorList = []
        # self.hub = hub()
        self.angleErrorRate = 0
        self.resetAngle()
        self.timer=MyTimer()
        self.actionStart = self.timer.getTime() / 1000
        self.step = 0

    def stats(self,df, i):
        # Compute the average of a column of data
        # df is a list of lists of floats, i is the index of the data within
        # df to average.
        ave = sum([l[i] for l in df]) / len(df)
        return ave



    def interpolate(self,df):
        # df is a list of list of float values
        # m is the slope of the line that best fits df, b is the y-intercept
        ave_x = self.stats(df, 0)
        ave_y = self.stats(df, 1)
        m = sum([l[0] * (l[1] - ave_y) for l in df]) / sum([l[0] * (l[0] - ave_x) for l in df])
        b = ave_y - m * ave_x

        return (m, b)




    def time(self):
        """Returns the time in seconds since the robot was started"""
        return (self.timer.getTime() / 1000)

    def setSpeed(self, speed=20):
        """Sets the speed of the robot"""
        self.speed = speed

    def getSpeed(self):
        """Returns the speed of the robot"""
        # r=self
        speed = (self.leftMotor.getSpeed() + self.rightMotor.getSpeed()) / 2
        return (speed)

    def isStalled(self, minDistance=300, maxDistance=600, speedReducedPercent=50, distPerTimePerSpeed=7, debug=False):
        """Returns True if the robot is stalled, False otherwise"""
        # r=self
        flag = False
        minDuration = abs(minDistance / self.speed / distPerTimePerSpeed)
        maxDuration = abs(maxDistance / self.speed / distPerTimePerSpeed)

        # distance/speed/7 =time
        t1 = self.time() - self.actionStart
        if (self.time() - self.actionStart > maxDuration) or (
                t1 > minDuration and abs(self.getSpeed()) < abs(self.speed * (100 - speedReducedPercent) / 100)):
            flag = True
        if flag:
            if debug:
                self.stop()
                print('minduration,maxduration,realDuration ', minDuration, maxDuration, t1)
                time.sleep(3)
        return flag

    def beep(self):
        """Beep the robot"""
        self.hub.speaker.beep()

    def stop(self,hold=False):
        """Stops the robot

        Args:
            hold (bool, optional): if True the robot will remain stopped after the command. Defaults to False."""
        self.leftMotor.stop(hold=hold)
        self.rightMotor.stop(hold=hold)

    def resetAngle(self, hardReset=True, debug=True):
        """Resets the angle of the robot

        Args:
            hardReset (bool, optional): if True the gyro will also will be reset. Defaults to True.
            debug (bool, optional): if True debugging informationswill be printed. Defaults to True.
        """
        # time.sleep(0.5)

        # need test
        currentOriginalAngle = self.getAngle(adjusted=False)
        targetAngle = round(currentOriginalAngle / 90.0) * 90
        error = currentOriginalAngle - targetAngle
        print('angle:', currentOriginalAngle, ' error:', error)
        if debug:
            print('currentOriginalAngle:', currentOriginalAngle)
            print('targetAngle:', targetAngle)
            print('identified error', error)
        self.angleErrorList.append((targetAngle,error))


        if hardReset:
            time.sleep(0.3)
            motion_sensor.reset_yaw(0)
            time.sleep(0.3)
            self.angle = 0
            self.lastAngle = 0
            self.zeroAngle = 0
            self.angleError = 0

    def calculateAngleErrorRate(self, debug=True):
        time.sleep(0.5)
        # need test
        currentOriginalAngle = self.getAngle()
        targetAngle = round(currentOriginalAngle / 90.0) * 90
        error = currentOriginalAngle - targetAngle
        if error == 0:
            self.angleErrorRate = 0
        elif targetAngle != 0:
            self.angleErrorRate = error / targetAngle

        if debug:
            print('currentOriginalAngle:', currentOriginalAngle)
            print('targetAngle:', targetAngle)
            print('identified error', error)

    def resetDistance(self):
        """Resets the distance of the robot"""
        # r=self
        self.leftMotor.resetDegrees()
        self.rightMotor.resetDegrees()

    def getDistance(self):
        """Returns the distance of the robot"""
        # r=self
        return (self.leftMotor.getDistance() + self.rightMotor.getDistance()) / 2

    def positionFromDistance(self,distanceOfElement,distancesOfAllElements=[689.056,602.5575, 512.3938,431.5152, 349.4149,260.473]):
        """We use this function to calculate the current position of an element when the element can be in different distances. """
        l1=[]
        i=1
        for item in distancesOfAllElements:
            l1.append([item,i])
            i+=1
        (m,k)=self.interpolate(l1)
        print('m,k',m,k)
        y=int(round(distanceOfElement*m+k))
        return(y)

    def getAngle(self, adjusted=True):
        """Returns the angle (heading) of the robot

        Args:
            adjusted (bool, optional): if True the angle will be fix drift errors. Defaults to True
        """
        self.lastAngle = self.angle
        self.angle = -motion_sensor.tilt_angles()[0]/10 #get_yaw_angle()
        # self.angleError = self.angle * self.angleErrorRate

        if self.angle - self.lastAngle < -180:
            self.zeroAngle = self.zeroAngle + 360
        elif self.angle - self.lastAngle > 180:
            self.zeroAngle = self.zeroAngle - 360
        if adjusted:
            val=self.angle+self.zeroAngle+self.angleError
        else:
            val=self.angle+self.zeroAngle
        # val = self.angle + self.zeroAngle
        targetAngle = round(val / 90.0) * 90

        return (val)

    def setAngle(self, targetAngle=20.0, maxAngle=90, minSpeed=10, maxSpeed=40, movingSpeed=0, steering=100,
                stopAtEnd=True):
        """Sets the angle of rotation of the robot

        Args:
            targetAngle (float): the target angle. Defaults to 20.0
            maxAngle (float, optional): the maximum angle. Defaults to 90
            minSpeed (int, optional): the minimum speed. Defaults to 10
            maxSpeed (int, optional): the maximum speed. Defaults to 40
            movingSpeed (int, optional): the speed to move at. Defaults to 0
            steering (int, optional): the speed to steer at. Defaults to 100
            stopAtEnd (bool, optional): if True the robot will stop at the end of the command. Defaults to True
        """
        currentAngle = self.getAngle()
        finalAngle = targetAngle

        error = (finalAngle - currentAngle)
        # print('current=',currentAngle,'target=',targetAngle)
        if error == 0:
            self.power = 0
        else:
            y = error / maxAngle * 90 * math.pi / 180
            sy = math.sin(y)
            v = sy * (maxSpeed - minSpeed)
            if sy != 0:
                v = v + sy / abs(sy) * minSpeed
            self.power = v

        if steering == 50:
            self.leftMotor.setSpeed(movingSpeed + self.power)
        elif steering==75:
            self.leftMotor.setSpeed(movingSpeed + self.power)
            self.rightMotor.setSpeed((movingSpeed - self.power)/2)
        elif steering==100:
            self.leftMotor.setSpeed(movingSpeed + self.power)
            self.rightMotor.setSpeed(movingSpeed - self.power)
        elif steering == -50:
            self.rightMotor.setSpeed(movingSpeed + self.power)
        elif steering == -75:
            self.leftMotor.setSpeed((movingSpeed - self.power)/2)
            self.rightMotor.setSpeed(movingSpeed + self.power)
        elif steering == -100:
            self.leftMotor.setSpeed(movingSpeed - self.power)
            self.rightMotor.setSpeed(movingSpeed + self.power)
        else:
            self.leftMotor.setSpeed(movingSpeed + self.power)
            self.rightMotor.setSpeed(movingSpeed - self.power)

        return (error)

    def steering(self, steering, power):
        """
        Converts steering & powerarguments to LeftPower,RightPower Values

        Parameters:
            steering: input steering
            power:Input Power
        return:
        (float,float): (leftPower,rightPower)
        """
        if type(power)!=type([]):
            power_left = power_right = power
            s = (50 - abs(float(steering))) / 50
            if steering >= 0:
                power_right *= s
                if steering > 100:
                    power_right = - power
            else:
                power_left *= s
                if steering < -100:
                    power_left = - power
            return (int(power_left), int(power_right))
        else:
            out=[]
            for p in power:
                power_left = power_right = p
                s = (50 - abs(float(steering))) / 50
                if steering >= 0:
                    power_right *= s
                    if steering > 100:
                        power_right = - p
                else:
                    power_left *= s
                    if steering < -100:
                        power_left = - p
                out.append((int(power_left), int(power_right)) )
            return (out)

    def steering2(self, steering, power):
        """
        Converts steering & powerarguments to LeftPower,RightPower Values

        Parameters:
            steering: Το steering εισόδου
            power:Το Power εισόδου
        return:
        (float,float): (leftPower,rightPower)
        """
        if steering >= 0:
            if steering > 100:
                power_right = 0
                power_left = power
            else:
                power_left = power
                power_right = power - ((power * steering) / 100)
        else:
            if steering < -100:
                power_left = 0
                power_right = power
            else:
                power_right = power
                power_left = power + ((power * steering) / 100)
        return (int(power_left), int(power_right))



    def calcSteering(self,powerLeft=50,powerRight=50):
        """Υπολογίζει τα power, steering για συγκεκριμμένα LeftPower, RightPower

        Args:
            powerLeft (int, optional): [description]. Defaults to 50.
            powerRight (int, optional): [description]. Defaults to 50.
        """
        if abs(powerRight)>abs(powerLeft):
            power=powerRight
            steering=(powerLeft/powerRight-1)*50
        elif abs(powerRight)<abs(powerLeft):
            power=powerLeft
            steering=(1-powerRight/powerLeft)*50
        elif powerRight>powerLeft:
            power=powerRight
            steering=(powerLeft/powerRight-1)*50
        else :
            power=powerLeft
            steering=(1-powerRight/powerLeft)*50

        if steering>100:
            steering=100
        if steering<-100:
            steering=-100

        return(power,steering)


    def turn(self, angle, steering=100, speed=0, acceptedError=2, autoCorrectAngle=90, stop=True,minSpeed=20, maxSpeed=50,
            debug=False):

        """Turn Robot using gyro sensor (not inside while loop)

        Args:
            angle (int): target angle
            steering (int): a value from -100 to 100. Defaults to 100.
            minSpeed (int, optional): minimum speed. Defaults to 20.
            maxSpeed (int, optional): maximum speed. Defaults to 40.
            acceptedError (int, optional): accepted error. Defaults to 1.
            autoCorrectAngle (int, optional): angle auto correction intervals. Defaults to 90.
            stop (bool, optional): stop movement at end. Defaults to True.
        """
        if speed!=0:
            maxSpeed=speed
            minSpeed=20*speed/abs(speed)
        startingAngle = self.getAngle()
        # print('currentAngle',currentAngle)
        finalAngle = round(startingAngle / autoCorrectAngle) * autoCorrectAngle + angle
        while True:
            currentAngle = self.getAngle()
            if finalAngle < currentAngle:
                sign = -1
            else:
                sign = 1
            x = self.setAngle(targetAngle=finalAngle, maxAngle=90 * sign, minSpeed=minSpeed, maxSpeed=maxSpeed,
                            steering=steering)
            # print('current angle',currentAngle,'finalAngle=',finalAngle,'error=', x)
            if abs(x) <= acceptedError:
                if stop:
                    self.stop()
                break
        # self.stop()
        if debug:
            print("turn completed (Gyro) (fromAngle,toAngle,real) Angle: (", startingAngle, ",", finalAngle, ",",
                self.getAngle(), ")", ' self.angle:', self.angle)

    def turnToHour(self, hour=3, angle=90,speed=70, acceptedError=2, autoCorrectAngle=90, stop=True,steering=50
            ,debug=False):

        """Turn Robot using pivot turn (not inside while loop)

        Args:
            hour (int): target hour
        """
        steering1=steering
        # angle1=abs(angle)
        speed1=abs(speed)
        if hour==3:
            angle1=abs(angle)
            steering1=abs(steering)
            speed1=abs(speed)
        elif hour==-3:
            angle1=-abs(angle)
            steering1=abs(steering)
            speed1=-abs(speed)
        elif hour==9:
            angle1=-abs(angle)
            steering1=-abs(steering)
            speed1=abs(speed)
        elif hour==-9:
            angle1=abs(angle)
            steering1=-abs(steering)
            speed1=-abs(speed)


        elif hour in (0,12):
            angle1=0
            steering1=abs(steering)
            speed1=abs(speed)

            startingAngle=self.getAngle()
            finalAngle = round(startingAngle / autoCorrectAngle) * autoCorrectAngle
            if startingAngle>finalAngle:
                steering1=-abs(steering)
            else:
                steering1=abs(steering)


        # elif hour==6:
        #    startingAngle=self.getAngle()
        #    finalAngle = round(startingAngle / autoCorrectAngle) * autoCorrectAngle
        #        if startingAngle>finalAngle:
        #            steering=-100
        #        else:
        #            steering=100

        #        angle1=0
        #        speed1=abs(speed)
        #    else:
        #        startingAngle=self.getAngle()
        #        finalAngle = round(startingAngle / autoCorrectAngle) * autoCorrectAngle
        #        if startingAngle>finalAngle:
        #            steering=50
        #        else:
        #            steering=-50
        #        angle1=0
        #        speed1=-abs(speed)



        if hour in (-3,3,9,-9,0,12):
            # print(angle1,steering1,speed1)
            self.turn(angle1,steering=steering1, speed=speed1, acceptedError=acceptedError, autoCorrectAngle=autoCorrectAngle, stop=stop, debug=debug)


    def followAngle(self, angle=0, speed=30, kp=4, autoCorrectAngle=90, acceleration=20 ,debug=False):
        """Tries to follow specific angle starting from nearest vertical line.

        Args:
            angle (int, optional): angle to follow. Defaults to 0.
            speed (int, optional): basic speed of movement. Defaults to 30.
            kp (float, optional): a factor that converts error to correction. Defaults to 3.5.
            autoCorrectAngle (int, optional): start measure angle intervals . Defaults to 90.
            debug (bool, optional): about printing information. Defaults to False.
        """
        r=self
        if self.step == 0:
            self.actionStart = self.timer.getTime() / 1000
            self.speed = speed
            self.pid = PID(KP=kp, KI=0.0, KD=0)
            # to be ready for isBlack()
            for i in range(9):
                self.color1.getLight()

            self.step += 1
            self.sumError = 0
            # currentAngle=self.getAngle()
            # need testing
            # currentAngle = self.getAngle() - self.angleError
            currentAngle = self.getAngle()
            self.finalAngle = currentAngle + angle
            if autoCorrectAngle != 0:
                self.finalAngle = round(currentAngle / int(autoCorrectAngle)) * int(autoCorrectAngle) + angle
                if debug:
                    print('current angle ', currentAngle, 'final ', self.finalAngle)
        else:
            self.step += 1
            x = r.getAngle()
            self.sumError += self.finalAngle - x
            correction = self.pid.getCorrection(set_point=self.finalAngle, state=x, dt=0.02, debug=False)

            if debug:
                if self.step % 600 == 0:
                    print('speed:', self.speed, 'angle:', x, 'correction:', correction)
                # time.sleep(1)
            else:
                t2 = self.timer.getTime() / 1000
                newSpeed = self.nextSpeed(speed=self.speed, acceleration=acceleration, t0=self.actionStart, t1=t2)
                r.leftMotor.setSpeed(speed=newSpeed + correction)
                r.rightMotor.setSpeed(speed=newSpeed - correction)

    def followAngleDone(self, fixAngle=False, debug=False):
        """This methos need to be called every time followAngle method is completed. (before break loop)

        Args:
            fixAngle (bool, optional): if we want to calibrate data from followAngle data. Defaults to False.
            debug (bool, optional): Prints some information.. Defaults to False.
        """

        if debug:
            end = self.timer.getTime() / 1000
            print('final angle:', self.finalAngle)
            # print('angleList:',self.angleList)
            print('Follow Angle dt=', (end - self.actionStart) / self.step, ' Average Error:',
                self.sumError / self.step)
            if self.sum1 != 0:
                print('avg angle:', self.sum2 / self.sum1)
            print('Angle Error:', self.angleError)
            print('leftMotor.getDistance():', self.leftMotor.getDistance())
            print('rightMotor.getDistance():', self.rightMotor.getDistance())
            print('Light List', self.color1.values)

        self.step = 0

    def followWhiteLane(self, angle=-5, speed=30, kp=1, kp2=2.5, port=1, autoCorrectAngle=90, logAngles=True):
        # FollowAngle Test Start
        r=self
        if self.step == 0:
            self.actionStart = self.timer.getTime() / 1000
            # self.angleList=[]
            self.angleError = 0
            self.speed = speed
            self.sumError = 0
            self.sum1 = 0
            self.sum2 = 0
            self.pid = PID(KP=kp, KI=0.0, KD=0)
            self.pid2 = PID(KP=kp2, KI=0, KD=0)
            self.step += 1
            self.startingAngle = self.getAngle()
            self.finalAngle = self.startingAngle + angle
            # self.finalAngle2=currentAngle+angle*3
            self.foundLine = False
            if autoCorrectAngle != 0:
                self.startingAngle = round(self.startingAngle / int(autoCorrectAngle)) * int(autoCorrectAngle)
                self.finalAngle = self.startingAngle + angle

        else:
            self.step += 1
            x = r.getAngle()

            self.sumError += self.finalAngle - x
            # add an extra angle if detect line
            if port == 1:
                # x2=-kp2*10*(100-r.color1.getLight())*leftRight/100*(100-abs(self.speed))/100
                x2 = r.color1.getLight()
            elif port == 2:
                # x2=-kp2*10*(100-r.color2.getLight())*leftRight/100*(100-abs(self.speed))/100
                x2 = r.color2.getLight()

            if x2 < 80:
                self.foundLine = True
                xx = 1 - abs((x2 - 50)) / 50
                self.sum1 += xx
                self.sum2 += xx * x

            self.finalAngle = self.startingAngle + angle - (3 - x2 // 26) * angle * kp2
            correction = self.pid.getCorrection(set_point=self.finalAngle, state=x, dt=0.02)
            r.leftMotor.setSpeed(speed=self.speed + correction)
            r.rightMotor.setSpeed(speed=self.speed - correction)

    def followWhiteLaneDone(self, fixAngle=False, debug=False):
        """This method need to be called every time followAngle method is completed. (before break loop)

        Args:
            fixAngle (bool, optional): if we want to calibrate data from followAngle data. Defaults to False.
            debug (bool, optional): Prints some information.. Defaults to False.
        """
        # if fixAngle:
        #    if len(self.angleList)>0:
        #        self.angleList.sort()
        #        med=self.angleList[len(self.angleList)//2]

        #        med1=med-round(med/90)*90

        #        print('med=',med,'med1=',med1)
        #        print('before angle error:',self.angleError)
        #        self.angleError-=round(med1)
        #        print('after angle error:',self.angleError)

        if debug:
            end = self.timer.getTime() / 1000
            print('final angle:', self.finalAngle)
            # print('angleList:',self.angleList)
            print('Follow Angle dt=', (end - self.actionStart) / self.step, ' Average Error:',
                self.sumError / self.step)
            if self.sum1 != 0:
                print('avg angle:', self.sum2 / self.sum1)
            print('Angle Error:', self.angleError)
            print('leftMotor.getDistance():', r.leftMotor.getDistance())
            print('rightMotor.getDistance():', r.rightMotor.getDistance())

        self.step = 0

    def followLine(self, sensorIndex=1, speed=30, leftRight=-1, target=60, kp=1.0, kd=1.5, ki=0, logAngles=True,
                debug=False):
        """Robot Follow a line using 1 color sensor

        Args:
            sensorIndex (int, optional): sensor index.(1 for color1, 2 color2,3 color3) Defaults to 1.
            speed (int, optional): The basic moving speed. Defaults to 20.
            leftRight (int, optional): -1 means from left side of line. 1 from right side. Defaults to -1.
            target (int, optional): target percent of light in line edge. Defaults to 60.
            kp (float, optional): factor that converts error to correction. Defaults to 1.
            kd (int, optional): another factor. Defaults to 1.5.
            ki (int, optional): also an other factor. Defaults to 0.
            logAngles (bool, optional): if we want to log gero angles (in order to calibrate gero sensor). Defaults to True.
            debug (bool, optional): if True displays some informations. Defaults to False.
        """
        r=self
        self.angle = self.getAngle()
        if self.step == 0:
            self.actionStart = self.timer.getTime() / 1000
            self.speed = speed
            self.pid = PID(KP=kp, KI=ki, KD=kd)
            self.step += 1
            # self.angleList=[]
            self.angleError = 0
            self.lastDistance=0
            self.sum1 = 0
            self.sum2 = 0
            angleData={}
        else:
            x=r.getAngle()
            dis=self.getDistance()
            if logAngles and dis>self.lastDistance+1:
                self.lastDistance=dis
                an=round(x)
                if an in self.angleData:
                    self.angleData[an]+=1
                else:
                    self.angleData[an]=1
                    
                # if len(self.angleList)<100:
                #     self.angleList.append(self.angle)
                # else:
                #     self.angleList[self.step % 100]=self.angle
                
            
            self.step += 1
            if sensorIndex == 1:
                x = self.color1.getLight()
                correction = self.pid.getCorrection(set_point=target, state=x, dt=0.02) * leftRight
            elif sensorIndex == 2:
                x = self.color2.getLight()
                correction = self.pid.getCorrection(set_point=target, state=x, dt=0.02) * leftRight
            elif sensorIndex == 12:
                x = self.color2.getLight() - r.color1.getLight()
                correction = self.pid.getCorrection(set_point=0, state=x, dt=0.02)
            else:
                print('Invalid sensorIndex', sensorIndex)
            if abs(self.pid._error) < 30:
                xx = 1 - abs(self.pid._error) / target
                self.sum1 += xx
                self.sum2 += xx * self.angle
            if debug:
                if self.step % 600 == 0:
                    print('speed:', self.speed, 'light:', x, 'correction:', correction)
                # time.sleep(1)
            else:
                self.leftMotor.setSpeed(speed=self.speed + correction)
                self.rightMotor.setSpeed(speed=self.speed - correction)

    def followLineDone(self, fixAngle=True, debug=False):
        """This methos need to be called every time followLine method is completed. (before break loop)

        Args:
            fixAngle (bool, optional): if we want to calibrate data from followAngle data.. Defaults to False.
            debug (bool, optional): Prints some information. Defaults to False.
        """
        end = self.timer.getTime() / 1000
        if fixAngle:
            if len(self.angleData) > 0:
                am=max(self.angleData.values())
                aMaxPos=list(self.angleData.values()).index(am)
                self.angleError-=list(self.angleData.keys())[aMaxPos]
            # if len(self.angleList)>0:
            #     self.angleList.sort()
            #     med=self.angleList[len(self.angleList)//2]

            #     med1=med-round(med/90)*90

            #     print('med=',med,'med1=',med1)
            #     print('before angle error:',self.angleError)
            #     self.angleError-=round(med1)
            #     print('after angle error:',self.angleError)

        if debug:
            print('Follow Line dt=', (end - self.actionStart) / self.step, 'steps:', self.step)
            if self.sum1 != 0:
                print('avg angle:', self.sum2 / self.sum1)
            print('leftMotor.getDistance():', self.leftMotor.getDistance())
            print('rightMotor.getDistance():', self.rightMotor.getDistance())

        self.step = 0

    def scanColors(self, scanningColors=[3], distanceBetweenColors=15):
        """This method scans colors and updates the list of colors detected (scanColorList) and the relative distances (scanDistances)."""

        # r=self

        if self.step3 == 0:
            self.lastScanDistance = 0
            self.step3 += 1
            self.scanColorList = []
            self.scanDistances = []
        self.scanColor = self.color3.getColor()
        self.scanDistance = self.getDistance()
        if self.scanColor in scanningColors:
            if abs(self.lastScanDistance - self.scanDistance) > distanceBetweenColors:
                self.scanColorList.append(self.scanColor)
                self.scanDistances.append(self.scanDistance)
            self.lastScanDistance = self.scanDistance

    def scanColorsDone(self):
        """This method need to be called every time scanColors method is completed. (before break loop)"""
        # r=self
        self.step3 = 0
        return self.scanColorList

    def waitUntilButton(self,checkColor1=False, checkGyro=True):
        """This method waits until the button is pressed.

        Args:
            checkColor1 (bool, optional): if we want to check color1. Defaults to False.
            checkGyro (bool, optional): if we want to check gyro. Defaults to True.
        """
        # r=self
        error = False
        if checkGyro:
            t0 = self.time()
            a0 = self.getAngle()
            time.sleep(3)
            a = self.getAngle()
            if abs(a - a0)/3 > 0.1:
                img = 'NO'
                error = True
        x = self.color1.getLight()
        if checkColor1:
            if x < 50:
                img = 'NO'
                error = True

        if error:
            print('a0/a/gyro error:',a0,a,abs(a - a0)/3,'per sec')
            raise SystemExit()
        # Wait for the left button to be pressed
        light_matrix.show_image(light_matrix.IMAGE_TRIANGLE_LEFT)

        print('time1:',self.time())
        while not button.pressed(button.LEFT) :
            pass

        self.timer.reset()
        print('time2:',self.time())


    def wait(self,step=1):
        """This method pause the program dispalying the step in the display. You have to press left or right butoon to continue.
        """
        # r=self
        # light_matrix.show_image(light_matrix.IMAGE_TARGET)
        if type(step)==type(5):
            msg=str(step)
        elif type(step)==type('a'):
            msg=step
        else:
            msg=''

        self.printMatrix(message=msg)
        if self.debug:

            while not button.pressed(button.LEFT) and not button.pressed(button.RIGHT) :
                pass


    def nextSpeed(self, speed=40, acceleration=20, t0=0, t1=1):
        """This method calculates the proper speed for a given accelaration and time."""
        minAbsSpeed = 20
        sign = speed / abs(speed)
        speed2 = minAbsSpeed + acceleration * (t1 - t0)
        newSpeed = sign * min(abs(speed), speed2)
        return (newSpeed)

    def printMatrix(self, message='-'):
        """This method displays a message to the display.

        Args:
            message (str): message to display. Defaults to '-'."""
        light_matrix.write(str(message))


    #    return colors

    def moveForDistance(self, speed=25, distance=100, angle=0,kp=4,slowDownAtEnd=True,acceleration=40 ,scanningColors=[], distanceBetweenColors=15,motorDegrees=[]):
        """This method moves the robot for a given distance.

        Args:
            speed (int, optional): speed of the robot. Defaults to 25.
            distance (int, optional): distance to move. Defaults to 100.
            angle (int, optional): angle (heading) to follow. Defaults to 0.
            slowDownAtEnd (bool, optional): if we want to slow down at the end of the move. Defaults to True.
            scanningColors (list, optional): list of colors to scan. Defaults to [].
            distanceBetweenColors (int, optional): distance between colors. Defaults to 15.
            motorDegrees (list, optional): If you want to control the angle of the motors (motor1, motor2) set the list of motor degrees. Defaults to []."""
        distance=abs(distance)
        slowDownDistance=0
        if slowDownAtEnd:
            slowDownDistance=abs(speed)*1
        self.resetDistance()
        colors = []
        while True:
            self.followAngle(speed=speed, angle=angle,acceleration=acceleration,kp=kp)
            if len(scanningColors) > 0:
                self.scanColors(scanningColors=scanningColors, distanceBetweenColors=distanceBetweenColors)

            if len(motorDegrees)>=1:
                if motorDegrees[0]!=None:
                    if abs(self.motor1.getDegrees()-motorDegrees[0])<=3:
                        self.motor1.stop()
                    else:
                        self.motor1.setDegrees(motorDegrees[0])
            if len(motorDegrees)>=2:
                if motorDegrees[1]!=None:
                    if abs(self.motor2.getDegrees()-motorDegrees[1])<=3:
                        self.motor2.stop()
                    else:
                        self.motor2.setDegrees(motorDegrees[1])


            if abs(self.getDistance()) > abs(distance):
                self.stop()
                if len(motorDegrees)>=1:
                    self.motor1.stop()
                if len(motorDegrees)>=2:
                    self.motor2.stop()

                self.followAngleDone()
                if len(scanningColors) > 0:
                    colors = self.scanColorsDone()
                break
            elif abs(self.getDistance()) > distance-slowDownDistance:
                # self.speed=20*speed/abs(speed) +(distance-abs(self.getDistance())) / slowDownDistance*(speed-20*speed/abs(speed))
                self.speed=15*speed/abs(speed)
        return colors






    def moveToWall(self, speed=-50, minDistance=300, maxDistance=1200, angle=0,kp=4, scanningColors=[],
                distanceBetweenColors=15):
        """This method moves the robot to the wall.

        Args:
            speed (int, optional): speed of the robot. Defaults to -50.
            minDistance (int, optional): minimum distance to move. Defaults to 300.
            maxDistance (int, optional): maximum distance to move. Defaults to 1200.
            angle (int, optional): angle (heading) to follow. Defaults to 0.
            scanningColors (list, optional): list of colors to scan. Defaults to [].
            distanceBetweenColors (int, optional): distance between colors. Defaults to 15.
        """

        self.resetDistance()
        colors = []
        while True:
            self.followAngle(speed=speed, angle=angle,kp=kp)
            if len(scanningColors) > 0:
                self.scanColors(scanningColors=scanningColors, distanceBetweenColors=distanceBetweenColors)

            if self.isStalled(minDistance=minDistance, maxDistance=maxDistance):
                self.stop()
                self.followAngleDone()
                if len(scanningColors) > 0:
                    colors = self.scanColorsDone()
                break
        return colors


    def moveToWhite(self, angle=0, speed=50, minDistance=10, distanceAfterWhite=0,useSensors=[1,2],fixAngle=True,whiteThreshold=98,kp=4,motorDegrees=[],debug=False ):
        """This method moves the robot until it reaches a white lane.

        Args:
            angle (int, optional): angle (heading) to follow. Defaults to 0.
            speed (int, optional): speed of the robot. Defaults to 50.
            minDistance (int, optional): minimum distance to move. Defaults to 10.
            distanceAfterWhite (int, optional): distance after white. Defaults to 0.
            useSensors (list, optional): list of sensors needs to get into white to complete the movement. Defaults to [1,2].
            fixAngle (bool, optional): if we want to fix the angle. Defaults to True.
            whiteThreshold (int, optional): white threshold. Defaults to 98.
            motorDegrees (list, optional): If you want to control the angle of the motors (motor1, motor2
            debug (bool, optional): if we want to display the debug messages. Defaults to False.
        """
        # r=self
        if len(useSensors)<2:
            fixAngle=False
        self.resetDistance()
        # for i in range(9):
        #    x=r.color1.getLight()
        reachedWhite = False
        correctionDone=False
        leftWhite=False
        rightWhite=False
        markDistance1 = 0
        markDistance2 = 0
        markDistance=0
        slowDownDistance=abs(speed)*1
        while True:
            if len(motorDegrees)>=1:
                if motorDegrees[0]!=None:
                    if abs(self.motor1.getDegrees()-motorDegrees[0])<=3:
                        self.motor1.stop()
                    else:
                        self.motor1.setDegrees(motorDegrees[0],speed=50)
            if len(motorDegrees)>=2:
                if motorDegrees[1]!=None:
                    if abs(self.motor2.getDegrees()-motorDegrees[1])<=3:
                        self.motor2.stop()
                    else:
                        self.motor2.setDegrees(motorDegrees[1],speed=50)


            x1 = self.color1.getLight()
            x2 = self.color2.getLight()
            dis=self.getDistance()
            self.followAngle(speed=speed, angle=angle,acceleration=30,kp=kp)
            if abs(dis)>=minDistance:
                if 1 not in useSensors:
                        leftWhite=True
                        markDistance=dis
                if 2 not in useSensors:
                        rightWhite=True
                        markDistance=dis
                # check for white in color1
                if not leftWhite:
                    if self.color1.isWhite(threshold=whiteThreshold):
                        leftWhite=True
                        markDistance1=dis
                # check for white in color2
                if not rightWhite:
                    if self.color2.isWhite(threshold=whiteThreshold):
                        rightWhite=True
                        markDistance2 = dis

                # if only one white during 30mm ignore this white

                if leftWhite and rightWhite:
                    # self.stop()
                    # print(markDistance1,markDistance2)
                    reachedWhite=True
                    markDistance=(markDistance1+markDistance2)/2
                    if abs(self.getDistance() - markDistance) >= distanceAfterWhite:
                        self.stop()
                        # print('=== 1 ===')
                        # print('m1,m2,dis:',markDistance1,markDistance2,self.getDistance(),dis)
                    elif abs(self.getDistance() - markDistance) > distanceAfterWhite-slowDownDistance:
                        self.speed=15*speed/abs(speed)
                        # print('=== 2 ===')

                elif leftWhite and not rightWhite:
                    if abs(dis-markDistance1)>40:
                        leftWhite=False
                        markDistance1=dis
                        # print('=== 3 ===')
                elif not leftWhite and rightWhite:
                    if abs(dis-markDistance2)>40:
                        # print('=== 4 ===')
                        rightWhite=False
                        markDistance2=dis
                # if both sensors reached white
                if fixAngle and reachedWhite and not correctionDone:
                    # print('=== 5 ===')
                    apenanti=markDistance2-markDistance1
                    targetAngle = round(self.getAngle() / 90.0) * 90
                    angleError=math.degrees(math.atan(apenanti/self.distanceBetweenColorSensors))
                    verticalDistance=0.5*self.distanceBetweenColorSensors*math.sin(math.radians(angleError))
                    self.angleError=self.angleError+angleError
                    self.angleErrorList.append((targetAngle,self.angleError))
                    if debug:
                        print('Real angle,Vertical Angle,Angle Error:',self.getAngle(),targetAngle,self.angleError)
                        print('angleErrorList:',self.angleErrorList)
                    # if len(self.angleErrorList)>2:
                    #    self.errorForAngle()

                    correctionDone=True

            if reachedWhite and abs(self.getDistance() - markDistance) >= distanceAfterWhite:
                self.stop()
                # print('=== 6 ===')
                if len(motorDegrees)>=1:
                    self.motor1.stop()
                if len(motorDegrees)>=2:
                    self.motor2.stop()


                self.followAngleDone()
                # print(markDistance1,markDistance2,self.getDistance())
                break
                # print(r.color1.values)



    def moveToLine(self, angle=0, speed=50, minDistance=10, distanceAfterLine=0,useSensors=[1,2],fixAngle=True,blackThreshold=40,kp=4 ,motorDegrees=[] ,debug=False):
        """
        This method moves the robot until it reaches a black line.

        Args:
            angle (int, optional): The angle to move the robot. Defaults to 0.
            speed (int, optional): The speed to move the robot. Defaults to 50.
            minDistance (int, optional): The minimum distance to move the robot. Defaults to 10.
            distanceAfterLine (int, optional): The distance after the line. Defaults to 0.
            useSensors ([int, int], optional): The sensors to use. Defaults to [1, 2].
            fixAngle (bool, optional): Whether to fix the angle. Defaults to True.
            blackThreshold (int, optional): The black threshold. Defaults to 40.
            motorDegrees (list, optional): If you want to control the angle of the motors (motor1, motor2)
            debug (bool, optional): if we want to display the debug messages. Defaults to False.
        """
        # r=self
        if len(useSensors)<2:
            fixAngle=False
        self.resetDistance()
        # for i in range(9):
        #    x=r.color1.getLight()
        reachedBlack = False
        correctionDone=False
        leftBlack=False
        rightBlack=False
        markDistance1 = 0
        markDistance2 = 0
        markDistance=0
        slowDownDistance=abs(speed)*1
        while True:
            if len(motorDegrees)>=1:
                if motorDegrees[0]!=None:
                    if abs(self.motor1.getDegrees()-motorDegrees[0])<=3:
                        self.motor1.stop()
                    else:
                        self.motor1.setDegrees(motorDegrees[0],speed=50)
            if len(motorDegrees)>=2:
                if motorDegrees[1]!=None:
                    if abs(self.motor2.getDegrees()-motorDegrees[1])<=3:
                        self.motor2.stop()
                    else:
                        self.motor2.setDegrees(motorDegrees[1],speed=50)


            x1 = self.color1.getLight()
            x2 = self.color2.getLight()
            dis=self.getDistance()
            self.followAngle(speed=speed, angle=angle,acceleration=30,kp=kp)
            if abs(dis)>=minDistance:
                if 1 not in useSensors:
                        leftBlack=True
                        markDistance=dis
                if 2 not in useSensors:
                        rightBlack=True
                        markDistance=dis
                # check for white in color1
                if not leftBlack:
                    if self.color1.isBlack(threshold=blackThreshold):
                        leftBlack=True
                        markDistance1=dis
                # check for white in color2
                if not rightBlack:
                    if self.color2.isBlack(threshold=blackThreshold):
                        rightBlack=True
                        markDistance2 = dis

                # if only one white during 30mm ignore this white

                if leftBlack and rightBlack:
                    # self.stop()
                    # print(markDistance1,markDistance2)
                    reachedBlack=True
                    markDistance=(markDistance1+markDistance2)/2
                    if abs(self.getDistance() - markDistance) >= distanceAfterLine:
                        self.stop()
                        # print('=== 1 ===')
                        # print('m1,m2,dis:',markDistance1,markDistance2,self.getDistance(),dis)
                    elif abs(self.getDistance() - markDistance) > distanceAfterLine-slowDownDistance:
                        self.speed=15*speed/abs(speed)
                        # print('=== 2 ===')

                elif leftBlack and not rightBlack:
                    if abs(dis-markDistance1)>40:
                        leftBlack=False
                        markDistance1=dis
                        # print('=== 3 ===')
                elif not leftBlack and rightBlack:
                    if abs(dis-markDistance2)>40:
                        # print('=== 4 ===')
                        rightBlack=False
                        markDistance2=dis
                # if both sensors reached white
                if fixAngle and reachedBlack and not correctionDone:
                    # print('=== 5 ===')
                    apenanti=markDistance2-markDistance1
                    targetAngle = round(self.getAngle() / 90.0) * 90
                    angleError=math.degrees(math.atan(apenanti/self.distanceBetweenColorSensors))
                    verticalDistance=0.5*self.distanceBetweenColorSensors*math.sin(math.radians(angleError))
                    self.angleError=self.angleError+angleError
                    self.angleErrorList.append((targetAngle,self.angleError))
                    if debug:
                        print('Real angle,Vertical Angle,Angle Error:',self.getAngle(),targetAngle,self.angleError)
                        print('angleErrorList:',self.angleErrorList)

                    # if len(self.angleErrorList)>2:
                    #    self.errorForAngle()

                    correctionDone=True

            if reachedBlack and abs(self.getDistance() - markDistance) >= distanceAfterLine:
                self.stop()
                # print('=== 6 ===')
                if len(motorDegrees)>=1:
                    self.motor1.stop()
                if len(motorDegrees)>=2:
                    self.motor2.stop()


                self.followAngleDone()
                if debug:
                    print('markDist1,markDist2,curDistance:',markDistance1,markDistance2,self.getDistance())
                break
                # print(r.color1.values)




    def moveParallel(self,distance=100,speed=30):
        """
        This method moves the robot in parallel.

        Args:
            distance (int, optional): The distance +/- to move the robot. Defaults to 100.
            speed (int, optional): The speed +/- to move the robot. Defaults to 30.
        """
        r=self
        distToGo=abs(distance)
        a=math.acos((r.wheelToWheelDistance-distToGo)/r.wheelToWheelDistance)/math.pi*180
        print('angle to turn:',a)
        a0=r.getAngle()
        if speed<0:
            if distance<0:
                steering0=-50
                angle0=a
            else:
                steering0=50
                angle0=-a
        else:
            if distance>0:
                steering0=50
                angle0=a
            else:
                steering0=-50
                angle0=-a
        r.turn(angle=angle0,steering=steering0,speed=speed,autoCorrectAngle=90)
        a1=r.getAngle()

        if round(a0 / 90) * 90 == round(a1 / 90 ) * 90:
            r.turn(angle=0,steering=-steering0,speed=speed,autoCorrectAngle=90)
        else:
            r.turn(angle=-90*angle0/abs(angle0),steering=-steering0,speed=speed,autoCorrectAngle=90)

    def errorForAngle(self):
        r=self
        targetAngle = round(self.getAngle() / 90.0) * 90
        k=r.interpolate(r.angleErrorList[1:])
        print('a,b:',k)
        print('currentError/calcError:',r.angleError,targetAngle*k[0]+k[1])

    def printStatus(self):
        """
        This method prints some informations about the robot."""
        pass



class ArchieTeam(object):
    timer=MyTimer()
    r=MyRobot()


class Alpha(ArchieTeam):
    r=MyRobot()

    def __init__(self,robot):
        r=robot
        self.r=r
        r.color1 = MyColorSensor(sensorPort='E', name='color1')
        r.color2 = MyColorSensor(sensorPort='A', name='color2')
        r.leftMotor = MyMotor(sensorPort='B', isInverted=True, name='Left Motor',degreesToMM=math.pi*56/360,speedPowerEq=(1.100517, -0.3730566))
        r.rightMotor = MyMotor(sensorPort='D', isInverted=False, name='Right Motor',degreesToMM=math.pi*56/360,speedPowerEq=(1.103455, -0.01870263))
        r.motor1 = MyMotor(sensorPort='C', isInverted=False, name='motor1')
        r.motor2 = MyMotor(sensorPort='F', isInverted=True, name='motor2')

        # set aproximate white rgb values in case the robot starts from brown
        r.color1.setRGB()
        r.color1.setRGB()

        liftPositions=[0]
        grabPositions=[0]
        r.wheelToWheelDistance = 145
        r.sensorToSensorDistance=110
        self.liftPositions=[0,0,510,40,570,100,500,1060]


        # self.grabPositions=[0,0,-250,0,-590]
        # self.grabPositions=[0,0,-365,0,-690]
        self.grabPositions=[0,0,-330,0,-330,0,-330]
        lastLiftPosition=0
        lastGrabPosition=0


async def main():
    # write your code here
    # Date 06/18/2023
    gc.enable()
    a = gc.mem_alloc()
    b = gc.mem_free()
    print('gc_memaloc:', a)
    print('gc_mem_free:', b)

    print('starting program')
    r = MyRobot()

    # Choose your team
    # ##########################################
    # wro=Alpha(r)
    r.leftMotor=MyMotor(sensorPort='A',isInverted=True,degreesToMM=3.14*56/360)
    r.rightMotor=MyMotor(sensorPort='E',isInverted=False,degreesToMM=3.14*56/360)
    r.color1=MyColorSensor(sensorPort='B')
    r.color2=MyColorSensor(sensorPort='F')
    

    # start coding here ==================================
    #r.waitUntilButton(checkGyro=True)    
 
    while True:
        r.followLine(sensorIndex=1,speed=30,leftRight=-1,kp=1)
        if r.getDistance()>=400:
            r.stop()
            r.followLineDone(fixAngle=True)
            break
  
  
        
    






    # finish coding here ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    totTime=int(r.time())
    print('total time:',r.time())
    for i in range (5):
        r.printMatrix(totTime)
        time.sleep(3)    
    r.moveForDistance(speed=-30,distance=100)
    wro.setMotorPositions(liftPosition=0,grabPosition=0)
    raise SystemExit()

runloop.run(main())


