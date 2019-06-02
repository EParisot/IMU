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

## RPi Raspbian : activate OpenGL (optional, mandatory with vpython) :
https://eltechs.com/how-to-enable-opengl-on-raspberry-pi/

I2C interface should be activated

## usage : 
```
python3 main.py [--v](vpython/OpenGL needed) [--r file.csv] [--o file.csv]

--v : vpython 3d box visualisation
--r : read csv file
--o : write csv file

file row format : "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ" 
```

