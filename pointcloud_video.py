import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import argparse

from pyquaternion import Quaternion
from pathlib import Path

from spliner3D import *

def custom_draw_geometry_with_camera_trajectory(pcd, render_option_path, camera_trajectory_path, outdir, record=False):
    custom_draw_geometry_with_camera_trajectory.index = -1
    custom_draw_geometry_with_camera_trajectory.trajectory = o3d.io.read_pinhole_camera_trajectory(camera_trajectory_path)
    custom_draw_geometry_with_camera_trajectory.vis = o3d.visualization.Visualizer()

    #set intrinsic parameters for view
    #(open3d does not care about x and y offset)
    f_factor = 0.6#0.6
    width = 1920
    height = 1080
    intrinsic_new = o3d.camera.PinholeCameraIntrinsic(width, height, width*f_factor, width*f_factor, width/2, height/2)
    intrinsic_new.intrinsic_matrix =  [[width*f_factor, 0.00, width/2] , [0.00, width*f_factor, height/2], [0.00, 0.00, 1.00]]

    for c in custom_draw_geometry_with_camera_trajectory.trajectory.parameters:
        c.intrinsic = intrinsic_new


    image_path = os.path.join(outdir, 'image')
    if not os.path.exists(image_path):
        os.makedirs(image_path)
    depth_path = os.path.join(outdir, 'depth')
    if not os.path.exists(depth_path):
        os.makedirs(depth_path)

    def move_forward(vis):
        # This function is called within the o3d.visualization.Visualizer::run() loop
        # The run loop calls the function, then re-render
        # So the sequence in this function is to:
        # 1. Capture frame
        # 2. index++, check ending criteria
        # 3. Set camera
        # 4. (Re-render)
        ctr = vis.get_view_control()
        glb = custom_draw_geometry_with_camera_trajectory
        if glb.index >= 0:
            print("Capture image {:06d}".format(glb.index))
            if(record):
                image = vis.capture_screen_float_buffer(True)
                plt.imsave(os.path.join(image_path, '{:06d}.png'.format(glb.index)),
                        np.asarray(image),
                        dpi=1)

                # depth_path_full = os.path.join(depth_path, '{:06d}.png'.format(glb.index))
                # vis.capture_depth_image(depth_path_full, do_render=False, depth_scale=1000)

                depth = vis.capture_depth_float_buffer(True)
                plt.imsave(os.path.join(depth_path, '{:06d}.png'.format(glb.index)),
                        np.asarray(depth),
                        dpi=1)

        glb.index = glb.index + 1
        if glb.index < len(glb.trajectory.parameters):
            # print("intrinsics: ", glb.trajectory.parameters[glb.index].intrinsic.intrinsic_matrix)
            # f_factor = 0.6
            # intrinsic_new = o3d.camera.PinholeCameraIntrinsic(1920, 1080, 1920*f_factor, 1920*f_factor, 1920/2, 1080/2)
            # intrinsic_new.intrinsic_matrix =  [[1920*f_factor, 0.00, 1920/2] , [0.00, 1920*f_factor, 1080/2], [0.00, 0.00, 1.00]]
            # glb.trajectory.parameters[glb.index].intrinsic = intrinsic_new

            ctr.convert_from_pinhole_camera_parameters(
                glb.trajectory.parameters[glb.index], allow_arbitrary=True)
        else:
            custom_draw_geometry_with_camera_trajectory.vis.register_animation_callback(None)
            if(record):
                print("This segfault will terminate the program: ")
                vis.destroy_window() #this will cause a segmentation fault, but at least the program terminates

        return False

    vis = custom_draw_geometry_with_camera_trajectory.vis

    # do not show viewer while recording
    # speeds up image capturing and keeps the correct width/height size
    if(record):
        vis.create_window(width=width, height=height, left=0, top=0, visible=False)
    else:
        vis.create_window()

    vis.add_geometry(pcd)
    vis.get_render_option().load_from_json(render_option_path)
    vis.register_animation_callback(move_forward)
    vis.run()
    vis.destroy_window()

#interpolates 1 position between keyframes
#run several times for more frames
def interpolate_cam_traj_file(traj_path, outdir):
    trajectory = o3d.io.read_pinhole_camera_trajectory(traj_path)
    fn = Path(traj_path).stem
    traj_file_new = os.path.join(outdir, fn + "_interpolated_linear.txt")

    # traj_file_old = os.path.join(outdir, "trajectory_old.txt")
    # trajectory = o3d.io.read_pinhole_camera_trajectory(traj_file_old)

    counter = 0
    timestamp = 0.0
    with open(traj_file_new, 'w') as file:
        for c in trajectory.parameters:
            # print("extrinsics: ", c.extrinsic)
            ext = c.extrinsic

            # ext_new = np.zeros((4,4))
            
            R = ext[:3,:3]
            t = ext[:3, 3]
            t = -np.dot(R.T, t)

            q = Quaternion(matrix=R.T, atol=1e-05, rtol=1e-06)

            if(counter == 0):
                file.write(f"{timestamp} {t[0]} {t[1]} {t[2]} {q[1]} {q[2]} {q[3]} {q[0]}\n")
                timestamp += 1.0
                ext_old = ext
                counter += 1
                continue

            R_old = ext_old[:3,:3]
            t_old = ext_old[:3, 3]
            t_old = -np.dot(R_old.T, t_old)
            q_old = Quaternion(matrix=R_old.T, atol=1e-05, rtol=1e-06)

            q_slerp = Quaternion.slerp(q_old, q, amount=0.5)
            t_inter = (t + t_old)/2.0

            # ext_new[:3,:3] = q_slerp.rotation_matrix
            # ext_new[:3, 3] = t_inter

            # print("ext_new: ", ext_new)

            #interpolated timestamp
            file.write(f"{timestamp} {t_inter[0]} {t_inter[1]} {t_inter[2]} {q_slerp[1]} {q_slerp[2]} {q_slerp[3]} {q_slerp[0]}\n")
            timestamp += 1.0

            file.write(f"{timestamp} {t[0]} {t[1]} {t[2]} {q[1]} {q[2]} {q[3]} {q[0]}\n")
            timestamp += 1.0

            ext_old = ext
            counter += 1

def interpolate_with_splines(traj_path, outdir, spline_num, draw_plot=False):
    trajectory = o3d.io.read_pinhole_camera_trajectory(traj_path)
    fn = Path(traj_path).stem
    traj_file_new = os.path.join(outdir, fn + "_interpolated_spline.txt")

    # anchorX = [0, -5, -5, 0]
    # anchorY = [0, 9, -5, 4]
    # anchorZ = [0, 3, -5, 9]
    anchorX = []
    anchorY = []
    anchorZ = []

    quat_slerp = []

    #get anchors
    counter = 0
    for c in trajectory.parameters:
            # print("extrinsics: ", c.extrinsic)
            ext = c.extrinsic

            # ext_new = np.zeros((4,4))
            
            R = ext[:3,:3]
            t = ext[:3, 3]
            t = -np.dot(R.T, t)

            q = Quaternion(matrix=R.T, atol=1e-05, rtol=1e-06)

            if(counter == 0):
                anchorX.append(t[0])
                anchorY.append(t[1])
                anchorZ.append(t[2])
                ext_old = ext
                counter += 1
                continue

            R_old = ext_old[:3,:3]
            t_old = ext_old[:3, 3]
            t_old = -np.dot(R_old.T, t_old)
            q_old = Quaternion(matrix=R_old.T, atol=1e-05, rtol=1e-06)

            for k in range(0, spline_num-1):
                span = k*(1.0/(spline_num-1))
                q_slerp = Quaternion.slerp(q_old, q, amount=span)
                quat_slerp.append(q_slerp)

            ext_old = ext
            counter += 1

            # print("t: ", t)

            anchorX.append(t[0])
            anchorY.append(t[1])
            anchorZ.append(t[2])

    # print("anchorX: ", anchorX)
    
    controls = getPoints3d(anchorX, anchorY, anchorZ)
    t = np.linspace(0, 1, spline_num)
    splines = spliner3d(anchorX, anchorY, anchorZ, controls, t)

    timestamp = 0.0
    with open(traj_file_new, 'w') as file:
        for i in range(0, len(splines)):
            Bix, Biy, Biz = splines[i]
            for j in range(0, spline_num-1):
                q_curr = quat_slerp[i*(spline_num-1)+j]
                #offset to align capture with video
                # file.write(f"{timestamp} {Bix[j]-0.015} {Biy[j]+0.065} {Biz[j]} {q_curr[1]} {q_curr[2]} {q_curr[3]} {q_curr[0]}\n")
                file.write(f"{timestamp} {Bix[j]} {Biy[j]} {Biz[j]} {q_curr[1]} {q_curr[2]} {q_curr[3]} {q_curr[0]}\n")
                timestamp += 1.0

    if(draw_plot):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        for i in range(0, len(splines)):
            Bix, Biy, Biz = splines[i]
            plt.plot(Bix, Biy, Biz)
            for j in range(spline_num-1):
                plt.plot(Bix[j], Biy[j], Biz[j], marker="o")

        for j in range(0, len(anchorX)):
            plt.plot(anchorX[j], anchorY[j], anchorZ[j], marker="x")

        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory', help='path to trajectory file')
    parser.add_argument('--record', help='record frames', action='store_true')
    parser.add_argument('--plot', help='plot interpolated trajectory', action='store_true')
    parser.add_argument('--linear', help='use linear interpolation', action='store_true')
    parser.add_argument('--mode', default='view', help='view or interpolate', choices=['view', 'interpolate'])
    parser.add_argument('--outdir', default='./outputs', help='output directory')
    parser.add_argument('--render', default='./config/renderoption.json', help='render settings')
    parser.add_argument('--pcd', default=None, help='path to pointcloud file')
    parser.add_argument('--splines', type=int, default=10, help='number of spline interpolations')

    args = parser.parse_args()

    # sample_data = o3d.data.DemoCustomVisualization()

    if(args.mode == "interpolate"):
        if(args.linear):
            interpolate_cam_traj_file(args.trajectory, args.outdir)
        else:
            interpolate_with_splines(args.trajectory, args.outdir, args.splines, args.plot)

    if(args.pcd == None):
        print("No pointcloud specified!")
        exit()
    if(args.mode == "view"):
        pcd = o3d.io.read_point_cloud(args.pcd)
        custom_draw_geometry_with_camera_trajectory(pcd, args.render, args.trajectory, args.outdir, args.record)


    # interpolate_cam_traj_file(sample_data.camera_trajectory_path)

    # pcd = o3d.io.read_point_cloud(sample_data.point_cloud_path)
    # custom_draw_geometry_with_camera_trajectory(
    #     pcd, sample_data.render_option_path, sample_data.camera_trajectory_path)
