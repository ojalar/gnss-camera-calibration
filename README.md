# Infrastructure Camera Calibration with GNSS for Vehicle Localisation

This repository demonstrates the traffic camera calibration approach presented in the manuscript "GNSS Calibration for Vehicle Localisation from Traffic Cameras".

## Installation
At the moment, this repository only depends on NumPy. Dependencies can be installed via:
```
pip install -r requirements.txt
```

## Usage
In order to run the calibration, you need a properly formatted data file (see below) and the image dimensions in pixels. Ground plane origo is chosen at the first provided coordinate. The calibration is executed as:
```
python3 main.py -d "path-to-data" -img_w "image width" -img_h "image height"
```
To run the demo:
```
python3 main.py -d data/demo_data.csv -img_w 1024 -img_h 768
```

## Data Format
The data file used for calibration must be a text file with each row formatted as such:
```
bb x1, bb y1, bb x2, bb y2, timestamp, latitude, longitude, ellipsoidal height
```
where bb refers to the vehicle bounding box, (x1,y1) is the upper left corner and (x2, y2) is the lower right corner, in pixels. Latitude [rad], longitude [rad], and ellipsoidal height [m] are the coordinates of the vehicle.

Example data row from the demo file:
```
653, 384, 701, 420, 1642061795.0, 1.050000055465346, 0.43497193356640795, 20.6
```
