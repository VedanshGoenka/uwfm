YMD (Yaw Moment Diagram)
========================
A yaw moment diagram visualizes the lateral acceleration and yaw moment envelope
of a car.

Vehicle Model
=============
The vehicle is modeled as a four wheeled car. Tire forces are determined using
Magic Formula 5.2 tire data.

Running the Program
===================
This program is written in Python 3.4 on Linux. The following instructions are
how I get the code to run on my machine. I have not tried running the
simulation in another environment.

Create environment and activate
-------------------------------
```bash
$ pyvenv --system-site-packages env
$ source env/bin/activate
```

Install required packages
-------------------------
```bash
$ pip install -r requirements.txt
```
This might take a while. NumPy and SciPy take a bit of time to compile.

Modify the simulation parameters
--------------------------------
Sample simulation parameters are in the `data` folder. There are three
configuration folders: `simulation`, `tire`, `vehicle`.

`data/simulation/generic_ymd.ini` is a sample configuration for the ymd
simulation parameters.

`data/tire/generic_tire.ini` is an empty configuration for the MF5.2 tire
coefficients. You will need to populate a tire configuration file with your own
tire coefficients.

`data/vehicle/generic_fsae.ini` is a sample vehicle configuration. This
file defines the vehicle mass, geometry, suspension and static setup.

Run the simulation
------------------
```bash
$ python ymd_calculator.py
usage: ymd_calculator.py [-h] -v VEHICLE_CONFIG -t TIRE_CONFIG -s
                         SIMULATION_CONFIG [-o OUTPUT]
``` 
Example
```bash
$ python ymd_calculator.py -v data/vehicle/generic_fsae.ini -t data/tire/generic_tire.ini -s data/simulation/generic_ymd.ini
```
 
