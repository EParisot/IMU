# IMU GY-85
Raspberry IMU datas visualiser

	acc : adxl345
	gyro : itg3200
	magneto : qmc5883L

![](capture.bmp)

## requirements :
```
python3 
smbus 
Adafruit_GPIO
vpython(optional)
```

## RPi Raspbian : activate I2C interface (mandatory)
```
sudo raspi-config
	-> interfaces = activate I2C
```

## RPi Raspbian : activate OpenGL (optional, mandatory with vpython) :
```
sudo raspi-config
	-> Advanced Options -> GL Driver = Full KMS
```

## Script usage : 
```
python3 main.py [--v](vpython/OpenGL needed) [--r file.csv] [--o file.csv]

--v : vpython 3d box visualisation
--o : output data to csv file
```

## Thread Class usage
```
import GY-85
import IMU
import time

imu = IMU(GY-85())
imu.start()
# let the thread start
time.sleep(0.5)
while True:
	print(imu.values)
# Will print eternaly until "imu.stop = True"
```

