# IMU GY-85
Raspberry IMU datas visualiser

	acc : adxl345
	gyro : itg3200
	magneto : qmc5883L

![](capture.bmp)

## requirements :
python3;
vpython(optional)

## activate OpenGL (optional, mandatory with vpython) :
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
python3 main.py [--v](vpython/OpenGL needed)
```

