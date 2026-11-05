"""ArUco pose estimation compatible with OpenCV 4.5 and newer."""

import cv2
import numpy as np


class ArucoPoseEstimator:
    def __init__(self, dictionary='DICT_4X4_50', marker_id=0, marker_size=0.12,
                 max_reprojection_error=2.0, min_edge_pixels=25.0):
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV ArUco is missing; install python3-opencv (contrib build)')
        if not dictionary.startswith('DICT_') or not hasattr(cv2.aruco, dictionary):
            raise ValueError('Unknown ArUco dictionary')
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if marker_id < 0 or marker_id >= len(self.dictionary.bytesList):
            raise ValueError('Marker ID is outside the dictionary')
        if any(not np.isfinite(v) or v <= 0 for v in
               [marker_size, max_reprojection_error, min_edge_pixels]):
            raise ValueError('Marker size and quality thresholds must be finite and positive')
        self.marker_id = marker_id
        self.max_error = max_reprojection_error
        self.min_edge = min_edge_pixels
        params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters')
                  else cv2.aruco.DetectorParameters_create())
        self.params = params
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, params)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)
        # Refine explicitly after detection. This gives identical behavior on
        # Humble/OpenCV 4.5 and Jazzy/OpenCV 4.6, whose Python 3.12 binding can
        # crash while assigning DetectorParameters.cornerRefinementMethod.
        self.refinement_criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01
        )
        h = marker_size / 2
        self.object_points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                                      dtype=np.float64)

    def estimate(self, image, camera_matrix, distortion):
        corners, ids, _ = (self.detector.detectMarkers(image) if self.detector else
                           cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.params))
        if ids is None:
            return None
        matches = np.flatnonzero(ids.reshape(-1) == self.marker_id)
        if len(matches) != 1:
            return None  # Duplicate IDs are ambiguous.
        points = corners[int(matches[0])].reshape(4, 2).astype(np.float32)
        gray = (cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                if image.ndim == 3 else image)
        cv2.cornerSubPix(
            gray, points.reshape(-1, 1, 2), (5, 5), (-1, -1), self.refinement_criteria
        )
        points = points.astype(np.float64)
        height, width = image.shape[:2]
        if (np.min(np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)) < self.min_edge
                or np.any(points < 4) or np.any(points[:, 0] >= width-4)
                or np.any(points[:, 1] >= height-4)):
            return None
        ok, rvec, tvec = cv2.solvePnP(self.object_points, points, camera_matrix, distortion,
                                    flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)) or tvec[2, 0] <= 0:
            return None
        projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, camera_matrix, distortion)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2)-points)**2, axis=1))))
        if error > self.max_error:
            return None
        result = np.eye(4)
        result[:3, :3] = cv2.Rodrigues(rvec)[0]
        result[:3, 3] = tvec.reshape(3)
        if np.dot(result[:3, 2], result[:3, 3]) >= 0:
            return None  # The printed front of the marker must face the camera.
        return result, points.reshape(1, 4, 2).astype(np.float32), error
