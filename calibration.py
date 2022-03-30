import numpy as np

# This class contains implementations for calibration procedures
class Calibrator:
    def __init__(self, calibration_pixels, calibration_coordinates):
        # calibration pixels: numpy array [[u,v], [u,v]...]
        # calibration coordinates (ENU): numpy array [[x, y], [x, y]...]
        self.calibration_pixels = calibration_pixels
        self.calibration_coordinates = calibration_coordinates
        # homography matrix (3x3)
        self.H = np.array([])
        
    # finds optimal homography matrix, utilises RANSAC
    def calibrate(self, n = int(1e3), t = 3, s = 4):
        # n: number of iterations in RANSAC
        # t: inlier threshold [m]
        # s: minimum number of samples for fitting

        # NOTE: this is a legacy functionality in numpy
        np.random.seed(42)
        
        # gnss coordinates are utilised as regular 2D coordinates
        # pixel coordinates are transformed to homogeneous 2D coordinates
        calibration_coordinates = self.calibration_coordinates
        calibration_pixels = self.calibration_pixels
        calibration_pixels = np.append(calibration_pixels, 
            np.ones((calibration_pixels.shape[0], 1)), axis = 1)

        # initialise for RANSAC looping 
        best_inliers_n = -1
        best_inliers_list = None
        best_H = None
        
        # RANSAC main loop
        for i in range(n):
            # randomly select s point correspondences
            idx = np.random.choice(np.arange(0,calibration_pixels.shape[0]), s, replace=False)
            fit_coordinates = calibration_coordinates[idx]
            fit_pixels = calibration_pixels[idx]
            
            # fit homography to s correspondences 
            H = self.fit_homography(fit_coordinates, fit_pixels)
            
            # find inliers supporting current fit
            inliers = []
            for i in range(calibration_pixels.shape[0]):
                # homogeneous ground plane estimate via multiplication
                coord_hat_hg = np.dot(H, calibration_pixels[i])
                # divide with scale parameter
                coord_hat = np.array((coord_hat_hg[0]/coord_hat_hg[2],
                    coord_hat_hg[1]/coord_hat_hg[2]))
                # error as distance between points
                error = np.linalg.norm(coord_hat - calibration_coordinates[i])
                if error < t:
                    inliers.append(i)
            
            fit_coordinates = calibration_coordinates[inliers]
            fit_pixels = calibration_pixels[inliers]
            
            # fit homography to all inliers 
            H = self.fit_homography(fit_coordinates, fit_pixels)
            
            # if number of inliers higher than any found previously, save result 
            if len(inliers) > best_inliers_n:
                best_inliers_n = len(inliers)
                best_inliers_list = inliers
                best_H = H
        
        # save and return the result with highest number of inliers
        self.H = best_H
        return self.H
    
    # fit homography based on given coordinates and pixels
    def fit_homography(self, coordinates, pixels):
        # coordinates: numpy array [[x,y], [x,y]...]
        # pixels: numpy array [[u,v,1], [u,v,1]...]
        A = []
        for i in range(pixels.shape[0]):
            A.append([0, 0, 0, *-pixels[i, :], 
                *coordinates[i, 1]*pixels[i, :]])
            A.append([*pixels[i, :], 0, 0, 0, 
                *-coordinates[i, 0]*pixels[i, :]])
        
        A = np.array(A)
        # solve homogenous least squares Ah = 0, when ||h|| = 1
        # solution is the eigenvector corresponding to minimum eigenvalue
        ATA = np.dot(A.T, A)
        eig_values, eig_vectors = np.linalg.eig(ATA)
        h = eig_vectors[:, np.argmin(eig_values)].real
        H = h.reshape((3,3))
        
        return H
    
    # compute the ground plane position of given image coordinates 
    def position(self, u, v):
        # u: horizontal image coordinate
        # v: vertical image coordinate 
        if self.H.size == 0:
            raise RuntimeError("You must perform calibration first") 
        # transform to homogeneous coordinates
        pixels = np.array([u, v, 1])
        # multiply with H to acquire ground plane estimate in homogeneous coordinates 
        coord_hat_hg = np.dot(self.H, pixels)
        # divide by scale parameter to acquire regular 2D coordinates
        coord_hat = np.array((coord_hat_hg[0]/coord_hat_hg[2], coord_hat_hg[1]/coord_hat_hg[2]))
        
        return coord_hat[0], coord_hat[1] 

