import cv2
import numpy as np
import os
import fnmatch

import sys
import argparse

def extractImages(pathIn, pathOut):
    vidcap = cv2.VideoCapture(pathIn)
    success,image = vidcap.read()
    count = 0
    success = True
    while success:
      success, image = vidcap.read()
      print('Read a new frame: ', count)
      frame_name = 'frame_{:0>6}.png'.format(count)
      cv2.imwrite(os.path.join(pathOut, frame_name), image)     # save frame as JPEG file
      count += 1

def frames_to_video(inputpath, outputpath, fps):
   image_array = []
   files = [f for f in os.listdir(inputpath) if os.path.isfile(os.path.join(inputpath, f))]
   files.sort(key = lambda x: int(x[0:-4]))
   size = (1920, 1080)
   for i in range(len(files)):
       img = cv2.imread(inputpath + files[i])
    #    size =  (img.shape[1],img.shape[0])
       img = cv2.resize(img,size)
       image_array.append(img)
   fourcc = cv2.VideoWriter_fourcc('D', 'I', 'V', 'X')
   out = cv2.VideoWriter(outputpath,fourcc, fps, size)
   for i in range(len(image_array)):
       out.write(image_array[i])
   out.release()

def blend_frames(pathFrames):
    '''
    blends frames starting with "frame_" with the corresponding number
    '''
    files = [f for f in os.listdir(pathFrames) if fnmatch.fnmatch(f, 'frame_*')]
    size = (1920, 1080)

    for file in files:
        print("file: ", file[6:])
        img1 = cv2.imread(pathFrames + file)
        img2 = cv2.imread(pathFrames + file[6:])

        img1 = cv2.resize(img1, size)
        img2 = cv2.resize(img2, size)

        dst = cv2.addWeighted(img1, 0.5, img2, 0.5, 0)
        frame_name = "blended_" + file[6:]

        cv2.imwrite(os.path.join(pathFrames, frame_name), dst)


# inputpath = 'folder path'
# outpath =  'video file path/video.mp4'
# fps = 29
# frames_to_video(inputpath,outpath,fps)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pathFrames", help="path to frame folder")
    parser.add_argument("--pathVideo", default="./outputs/video.mp4",help="path to video")
    parser.add_argument('--fps', type=int, default=5, help='frames per second')
    parser.add_argument('--mode', default='frames', help='video, frames or blend', choices=['video', 'frames', 'blend'])
    args = parser.parse_args()
    print(args)

    if(args.mode=="blend"):
        blend_frames(args.pathFrames)

    if(args.mode=="frames"):
        extractImages(args.pathVideo, args.pathFrames)

    if(args.mode=="video"):
        fps = args.fps
        frames_to_video(args.pathFrames, args.pathVideo, fps)