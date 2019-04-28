# IMU GY-85
Raspberry IMU datas visualiser

	acc : adxl345
	gyro : itg3200
	magneto : qmc5883L

![](capture.bmp)

## requirements :
python3;
vpython(optional)

## RPi Raspbian : activate OpenGL (optional, mandatory with vpython) :
https://eltechs.com/how-to-enable-opengl-on-raspberry-pi/

## Adafruit_Python_GPIO installation (mandatory) :
```
sudo apt-get update
sudo apt-get install build-essential python-pip python-dev python-smbus
cd Adafruit_Python_GPIO
sudo python3 setup.py install
```

## usage : 
```
python3 main.py [--v](vpython/OpenGL needed) [--r file.csv] [--o file.csv]

--v : vpython 3d box visualisation
--r : read csv file
--o : write csv file

file row format : "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ" 
```

