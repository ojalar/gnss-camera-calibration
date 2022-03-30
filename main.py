import numpy as np
import parser
import calibration
import argparse
import sys

# Main executable for demonstrating the calibration procedure

def main(data_path, w, h):
    # acquire data from calibration files 
    car_data = parser.parse_autocalibration_file(data_path, w, h)
    # parse to variables
    calibration_coords = car_data[0]
    calibration_coords_ecef = car_data[1]
    calibration_coords_enu = car_data[2]
    calibration_bbs = car_data[3]
    origo_coords = car_data[4]
    origo_coords_ecef = car_data[5]
    
    # choose centre point of bounding box to represent car image location
    calibration_pixels = (calibration_bbs[:,:2] + calibration_bbs[:,2:4])/2
    # transform ENU gnss coordinates to ground plane by removing z-component
    calibration_coords_2d = calibration_coords_enu[:, :2]
    
    # calibrate 
    camera = calibration.Calibrator(calibration_pixels,
            calibration_coords_2d)
    H = camera.calibrate()
    
    # calculate estimation error on random point for demonstration
    rnd_id = np.random.randint(calibration_pixels.shape[0])
    rnd_img_point = calibration_pixels[rnd_id]
    estimate = camera.position(*rnd_img_point)
    err = np.linalg.norm(estimate - calibration_coords_2d[rnd_id])
    
    print("Origo set at:")
    print("- Geodetic: ", origo_coords)
    print("- ECEF: ", origo_coords_ecef) 
    print("---")
    print("Fitted Homography matrix (image plane to ground plane):")
    print(H)
    print("---")
    print("Measurement error [m] on a random calibration image point (id {}):".format(rnd_id))
    print(err)
    
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--data", help="path to calibration data file")
    ap.add_argument("-img_w", "--image_width", help="image width in pixels")
    ap.add_argument("-img_h", "--image_height", help="image height in pixels")
    args = vars(ap.parse_args())

    if not args.get("data", False):
        print("No path provided to calibration data file")
        sys.exit()
    if not args.get("image_width", False):
        print("No image width provided")
        sys.exit()
    if not args.get("image_height", False):
        print("No image height provided")
        sys.exit()

    main(args.get("data"), int(args.get("image_width")), int((args.get("image_height"))))
