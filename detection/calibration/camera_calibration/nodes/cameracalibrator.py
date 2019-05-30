#!/usr/bin/python
import cv2
import functools
import message_filters
import os
import rospy
from camera_calibration.camera_calibrator import OpenCVCalibrationNode
from camera_calibration.calibrator import ChessboardInfo, Patterns
from message_filters import ApproximateTimeSynchronizer


def main():
    from optparse import OptionParser, OptionGroup
    parser = OptionParser("%prog --size SIZE1 --square SQUARE1 [ --size SIZE2 --square SQUARE2 ]",
                          description=None)
    parser.add_option("-c", "--camera_name",
                     type="string", default='autoware_camera_calibration',
                     help="name of the camera to appear in the calibration file")
    parser.add_option("-o", "--output",
                     type="string", default="yaml",
                     help="type of output - 'yaml' or 'tar'")
    parser.add_option("-d", "--detection",
                     type="string", default="cv2",
                     help="Chessboard detection algorithm, OpenCV2 or Matlab (python matlab engine) - 'cv2', 'matlab'")
    group = OptionGroup(parser, "Chessboard Options",
                        "You must specify one or more chessboards as pairs of --size and --square options.")
    group.add_option("-p", "--pattern",
                     type="string", default="chessboard",
                     help="calibration pattern to detect - 'chessboard', 'circles', 'acircles'")
    group.add_option("-s", "--size",
                     action="append", default=[],
                     help="chessboard size as NxM, counting interior corners (e.g. a standard chessboard is 7x7)")
    group.add_option("-q", "--square",
                     action="append", default=[],
                     help="chessboard square size in meters")
    group.add_option("--min_samples",
                     type="int", default=40,
                     help="defines the minimum number of samples before allowing to calibrate regardless of the status")
    parser.add_option_group(group)
    group = OptionGroup(parser, "ROS Communication Options")
    group.add_option("--approximate",
                     type="float", default=0.0,
                     help="allow specified slop (in seconds) when pairing images from unsynchronized stereo cameras")
    group.add_option("--no-service-check",
                     action="store_false", dest="service_check", default=True,
                     help="disable check for set_camera_info services at startup")
    parser.add_option_group(group)
    group = OptionGroup(parser, "Calibration Optimizer Options")
    group.add_option("--fix-principal-point",
                     action="store_true", default=False,
                     help="fix the principal point at the image center")
    group.add_option("--fix-aspect-ratio",
                     action="store_true", default=False,
                     help="enforce focal lengths (fx, fy) are equal")
    group.add_option("--zero-tangent-dist",
                     action="store_true", default=False,
                     help="set tangential distortion coefficients (p1, p2) to zero")
    group.add_option("-k", "--k-coefficients",
                     type="int", default=3, metavar="NUM_COEFFS",
                     help="number of radial distortion coefficients to use (up to 6, default %default)")
    group.add_option("--disable_calib_cb_fast_check", action='store_true', default=False,
                     help="uses the CALIB_CB_FAST_CHECK flag for findChessboardCorners")
    parser.add_option_group(group)

    options, args = parser.parse_args()

    if len(options.size) != len(options.square):
        parser.error("Number of size and square inputs must be the same!")

    if options.detection == "cv2":
        print('Using OpenCV 2 for chessboard corner detection')
    elif options.detection == "matlab":
        print('Using matlab for chessboard corner detection')
    else:
        print('Unrecognized detection method %s, defaulting to OpenCV 2' % options.detection)
        options.detection = "cv2"

    if options.output == "yaml":
            print('Saving as autoware yaml')
    elif options.output == "tar":
            print('Saving as tar')
    else:
            print('Unrecognized output method %s, defaulting to Autoware Yaml' % options.output)
            options.output = "yaml"

    if not options.square:
        options.square.append("0.108")
        options.size.append("8x6")

    boards = []
    for (sz, sq) in zip(options.size, options.square):
        size = tuple([int(c) for c in sz.split('x')])
        boards.append(ChessboardInfo(size[0], size[1], float(sq)))

    if options.approximate == 0.0:
        sync = message_filters.TimeSynchronizer
    else:
        sync = functools.partial(ApproximateTimeSynchronizer, slop=options.approximate)

    num_ks = options.k_coefficients

    calib_flags = 0
    if options.fix_principal_point:
        calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if options.fix_aspect_ratio:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if options.zero_tangent_dist:
        calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if (num_ks > 3):
        calib_flags |= cv2.CALIB_RATIONAL_MODEL
    if (num_ks < 6):
        calib_flags |= cv2.CALIB_FIX_K6
    if (num_ks < 5):
        calib_flags |= cv2.CALIB_FIX_K5
    if (num_ks < 4):
        calib_flags |= cv2.CALIB_FIX_K4
    if (num_ks < 3):
        calib_flags |= cv2.CALIB_FIX_K3
    if (num_ks < 2):
        calib_flags |= cv2.CALIB_FIX_K2
    if (num_ks < 1):
        calib_flags |= cv2.CALIB_FIX_K1

    pattern = Patterns.Chessboard
    if options.pattern == 'circles':
        pattern = Patterns.Circles
    elif options.pattern == 'acircles':
        pattern = Patterns.ACircles
    elif options.pattern != 'chessboard':
        print('Unrecognized pattern %s, defaulting to chessboard' % options.pattern)

    if options.disable_calib_cb_fast_check:
        checkerboard_flags = 0
    else:
        checkerboard_flags = cv2.CALIB_CB_FAST_CHECK

    rospy.init_node('cameracalibrator')
    node = OpenCVCalibrationNode(boards, options.service_check, sync, calib_flags, pattern, options.camera_name,
                                 options.detection, options.output, min_good_enough=options.min_samples, checkerboard_flags=checkerboard_flags)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()