import numpy as np

# This file contains functions for processing the calibration files
    
# Convert geodetic coordinates to ECEF coordinates
def convert_geodetic_to_ECEF(coord):
    # Convert geodetic WGS84 coordinates to ECEF coordinates
    lat = coord[0]
    lon = coord[1]
    h = coord[2]
    a = 6378137.0 # m
    b = 6356752.3 # m
    e = np.sqrt(1 - (b/a)**2)
    N = a/(np.sqrt(1 - (e**2)*(np.sin(lat)**2)))
    
    X = (N + h)*np.cos(lat)*np.cos(lon)
    Y = (N + h)*np.cos(lat)*np.sin(lon)
    Z = ((b/a)**2*N + h)*np.sin(lat)

    return X, Y, Z
    
# Converts ECEF coordinates to ENU coordinates, origo set at reference point
# defined as ref_coord
def convert_ECEF_to_ENU(coord, ref_coord, ref_geodetic):
    lat_ref = ref_geodetic[0]
    lon_ref = ref_geodetic[1]
    diff = np.array(coord) - np.array(ref_coord)
    transform = np.array([[-np.sin(lon_ref), np.cos(lon_ref), 0], 
        [-np.sin(lat_ref)*np.cos(lon_ref), -np.sin(lat_ref)*np.sin(lon_ref), np.cos(lat_ref)], 
        [np.cos(lat_ref)*np.cos(lon_ref), np.cos(lat_ref)*np.sin(lon_ref), np.sin(lat_ref)]])
    
    coords_enu = np.dot(transform, diff)

    return coords_enu

    
# Parse data from an autocalibration file
# file format: 
# bb x1, bb y1, bb x2, bb y2, timestamp, car latitude, car longitude, car ellipsoidal height
def parse_autocalibration_file(data_path, img_w, img_h):
    # read data from file
    data = np.genfromtxt(data_path, delimiter = ',')
    
    # acquire bounding boxes (x1,y1,x2,y2)
    calibration_bbs = data[:,:4]
    # translate coordinate system to image center, and mirror vertical components
    calibration_bbs[:,0] = calibration_bbs[:,0] - img_w/2
    calibration_bbs[:,1] = img_h/2 - calibration_bbs[:,1]
    calibration_bbs[:,2] = calibration_bbs[:,2] - img_w/2
    calibration_bbs[:,3] = img_h/2 - calibration_bbs[:,3]
    
    # acquire gnss coordinates
    calibration_coords = data[:,5:]
    # choose ENU origo at first coordinate
    origo_coords = calibration_coords[0, :]
    origo_coords_ecef = convert_geodetic_to_ECEF(origo_coords)
    # transform all coordinates to ECEF and ENU
    calibration_coords_ecef = np.array([convert_geodetic_to_ECEF(s) for s in calibration_coords])
    calibration_coords_enu = np.array([convert_ECEF_to_ENU(s, origo_coords_ecef,
        origo_coords) for s in calibration_coords_ecef])
    
    return calibration_coords, calibration_coords_ecef, calibration_coords_enu,\
        calibration_bbs, origo_coords, origo_coords_ecef

