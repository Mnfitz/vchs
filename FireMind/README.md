# FireMind GPS Tracker
FireMind is the brians of Project Phoenix, and requires two Motieno MEGAS (recieving/transmitting) and one Azimuth/Elevation motor to
function correctly. FireMind is a collection of programs designed to send, recieve, and process GPS coordinates and aim the motor mount,
so that the mount can track a moving target. Currently, as we only have one GPS tracking device, we are unable to track the recieving
Moteino. This means that if the reciever is attached to the motor, the mount needs to be kept stationary during an actual flight. The
target Motieno only requires FireMind_Target in order to work, and is intended to be mobile. It is important that all files EXCEPT
FireMind_Target are on the base station Motieno MEGA, and that the motor does not move.

Example of a Moteinop MEGA:

![alt text](https://github.com/Mnfitz/vchs/blob/master/FireMind/Assets/MoteinoMega.jpg "Moteino MEGA") 

Short Overview of Each Program:
1. FireMind_Recv: Takes in GPS coordinates sent from the target. Coordinates are sent via radio in floating point format. Note: Natural
data limitations reduce the precision of floating points sent; increasing the number of decimal places sent will lead to an increase
in accuracy. Please investigate.

2. FireMind_Calc: Turns the floating point numbers into XYZ (Left/Right, Up/Down, Forward/Back) and LLA (Latitude, Longtiude, Altitude)
Coordinates, which convey useful information to FireMind_Aim. By comparing the XYZ coordinates of the base and target, which can give
the horizontal and vertical bearings of the target, which can be used to aim the motor mount. FireMind_Calc relies heavily on 
FireMind_GpsCoords, as it contains the constructors for the GpsCoords and GpsXYZ classes. 

3. FireMind_GpsCoords: Defines GpsCoords and GpsXYZ classes which are used in FireMind_Calc. Both classes take in three floating point 
numbers as its data, which represent the coordinate position of the target. However, in GpsXYZ, it is implied that the data has been 
processed from LLA format (Earth Centered Earth Fixed) into XYZ format. 

4. Firemind_Aim: Will eventually process the bearings into aiming commands for the motor mount; currently only prints the values of the
bearings. 

5. FireMind_Main: Calls upon the various files used in FireMind, as mentioned above.

6. FireMind_Target: Gets GPS signals from nearby satellites and transmits them in floating point format. The first coordinate recieved
is inferred to be that of the base station, so you should keep the target next to the base until it begins to recieve GPS coordinates. 
Currently, the effective range of the target's radio has been 11 feet, so it would be important to find ways of increasing range for an 
actual flight. This is not a problem for ground testing, as you can carry around both the target and reciever, since the reciever only
updates its position once. If you can imagine there to be an invisible motor mount in the location of the first coordinate recieved, you
can test the bearing by moving the target relative to the "ghost motor mount." This method will not work in real life, as the reciever and the motor will have to be together, to form the base station.

```
# Location of the sketch (.ino) for "Base" Moteino
./vchs/FireMind/FireMind.ino

# Location of the sketch (.ino) for "Target" Moteino
./vchs/FireMind/FireMind_Target/FireMind_Target.ino
```

Ideas for the Future:
Increase range of transmission for FireMind_Target. Find new system of sending cooorcinates in order to increase accuracy of bearings.
Find a way to send bearings to the motor. Create method to find distance (range) of the base to the target, for the camera focussing
portion of the project. Find way to send commands from base station to the target.
