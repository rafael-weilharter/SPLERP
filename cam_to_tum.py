import numpy as np
import os
import fnmatch
import argparse

from pyquaternion import Quaternion

def read_cam_file(filename):
    # print("filename: ", filename)
    with open(filename) as f:
        lines = f.readlines()
        lines = [line.rstrip() for line in lines]
    # extrinsics: line [1,5), 4x4 matrix
    extrinsics = np.fromstring(' '.join(lines[1:5]), dtype=np.float32, sep=' ').reshape((4, 4))
    # intrinsics: line [7-10), 3x3 matrix
    intrinsics = np.fromstring(' '.join(lines[7:10]), dtype=np.float32, sep=' ').reshape((3, 3))

    return intrinsics, extrinsics

def convert_tat_to_tum(cam_path, traj_file):

    # _, _, files = next(os.walk(cam_path))
    files = [f for f in os.listdir(cam_path) if fnmatch.fnmatch(f, '*_cam.txt')]
    file_count = len(files)

    print(f"Found {file_count} files!")

    timestamp = 0.0
    failed = 0
    with open(traj_file, 'w') as file:
        for i in range(0, file_count):
            proj_mat_filename = os.path.join(cam_path, '{:0>8}_cam.txt'.format(i))
            intrinsics, ext = read_cam_file(proj_mat_filename)

            # if(i <= 40):
            #     R = ext[:3,:3]

            R = ext[:3,:3]

            t = ext[:3, 3]

            t = -np.dot(R.T, t)

            # t[0] = t[0]
            # t[1] = t[1]
            # t[2] = t[2]

            # print("R: ", R)
            try:
                q = Quaternion(matrix=R.T, atol=1e-05, rtol=1e-06) #reduce tolerances

                file.write(f"{timestamp} {t[0]} {t[1]} {t[2]} {q[1]} {q[2]} {q[3]} {q[0]}\n")
                # file.write(f"{timestamp} {t[0]} {t[1]} {t[2]} {q[0]} {q[1]} {q[2]} {q[3]}\n")
                timestamp += 1.0
            except:
                failed += 1
                # print("FAILED!")

    print("failed: ", failed)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cams", help="path to cam folder")
    parser.add_argument("--output", default="./trajectory_tat.txt",help="path of output txt file")
    args = parser.parse_args()

    convert_tat_to_tum(args.cams, args.output)
