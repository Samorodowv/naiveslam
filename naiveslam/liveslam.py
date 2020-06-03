#!/usr/bin/env python3
import os
import sys

sys.path.append("lib/macosx")
sys.path.append("lib/linux")

import time
import cv2
from display import Display2D, Display3D
from frame import Frame, match_frames
import numpy as np
import g2o
from pointmap import Map, Point
from helpers import triangulate, add_ones
from vidgear.gears.stabilizer import Stabilizer

np.set_printoptions(suppress=True)

class SLAM(object):
  def __init__(self, W, H, K):
    self.mapp = Map()

    self.W, self.H = W, H
    self.K = K

  def process_frame(self, img, pose=None, verts=None):
    start_time = time.time()
    assert img.shape[0:2] == (self.H, self.W)
    frame = Frame(self.mapp, img, self.K, verts=verts)

    if frame.id == 0:
      return

    f1 = self.mapp.frames[-1]
    f2 = self.mapp.frames[-2]

    idx1, idx2, Rt = match_frames(f1, f2)

    # esli tochka byla na pred cadre - nabludaem
    for i,idx in enumerate(idx2):
      if f2.pts[idx] is not None and f1.pts[idx1[i]] is None:
        f2.pts[idx].add_observation(f1, idx1[i])

    if frame.id < 5 or True:
      # fixiruem nachalnye pozicii
      f1.pose = np.dot(Rt, f2.pose)
    else:
      velocity = np.dot(f2.pose, np.linalg.inv(self.mapp.frames[-3].pose))
      f1.pose = np.dot(velocity, f2.pose)

    if pose is None:
      pose_opt = self.mapp.optimize(local_window=1, fix_points=True)
      print("Pose:     %f" % pose_opt)
    else:
      f1.pose = pose

    sbp_pts_count = 0

    if len(self.mapp.points) > 0:
      map_points = np.array([p.homogeneous() for p in self.mapp.points])
      projs = np.dot(np.dot(self.K, f1.pose[:3]), map_points.T).T
      projs = projs[:, 0:2] / projs[:, 2:]

      good_pts = (projs[:, 0] > 0) & (projs[:, 0] < self.W) & \
                 (projs[:, 1] > 0) & (projs[:, 1] < self.H)

      for i, p in enumerate(self.mapp.points):
        if not good_pts[i]:
          continue
        if f1 in p.frames:
          continue
        for m_idx in f1.kd.query_ball_point(projs[i], 2):
          if f1.pts[m_idx] is None:
            b_dist = p.orb_distance(f1.des[m_idx])
            if b_dist < 64.0:
              p.add_observation(f1, m_idx)
              sbp_pts_count += 1
              break

    good_pts4d = np.array([f1.pts[i] is None for i in idx1])

    pts4d = triangulate(f1.pose, f2.pose, f1.kps[idx1], f2.kps[idx2])
    good_pts4d &= np.abs(pts4d[:, 3]) != 0
    pts4d /= pts4d[:, 3:]       

    new_pts_count = 0
    for i,p in enumerate(pts4d):
      if not good_pts4d[i]:
        continue

      """
      r1 = np.dot(f1.pose[:3, :3], add_ones(f1.kps[idx1[i]]))
      r2 = np.dot(f2.pose[:3, :3], add_ones(f2.kps[idx2[i]]))
      parallax = r1.dot(r2) / (np.linalg.norm(r1) * np.linalg.norm(r2))
      if parallax >= 0.9998:
        continue
      """

      pl1 = np.dot(f1.pose, p)
      pl2 = np.dot(f2.pose, p)
      if pl1[2] < 0 or pl2[2] < 0:
        continue

      pp1 = np.dot(self.K, pl1[:3])
      pp2 = np.dot(self.K, pl2[:3])


      pp1 = (pp1[0:2] / pp1[2]) - f1.kpus[idx1[i]]
      pp2 = (pp2[0:2] / pp2[2]) - f2.kpus[idx2[i]]
      pp1 = np.sum(pp1**2)
      pp2 = np.sum(pp2**2)
      if pp1 > 2 or pp2 > 2:
        continue

      try:
        color = img[int(round(f1.kpus[idx1[i],1])), int(round(f1.kpus[idx1[i],0]))]
      except IndexError:
        color = (255,0,0)
      pt = Point(self.mapp, p[0:3], color)
      pt.add_observation(f2, idx2[i])
      pt.add_observation(f1, idx1[i])
      new_pts_count += 1

    print("Adding:   %d new points, %d search by projection" % (new_pts_count, sbp_pts_count))

    if frame.id >= 4 and frame.id%5 == 0:
      err = self.mapp.optimize() #verbose=True)
      print("Optimize: %f units of error" % err)

    print("Map:      %d points, %d frames" % (len(self.mapp.points), len(self.mapp.frames)))
    print("Time:     %.2f ms" % ((time.time()-start_time)*1000.0))
    print(np.linalg.inv(f1.pose))


if __name__ == "__main__":

  disp2d, disp3d = None, None
    
  if os.getenv("HEADLESS") is None:
    disp3d = Display3D()

  cap = cv2.VideoCapture(0)

  W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
  H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

  CNT = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
  F = float(os.getenv("F", "1000"))
  if os.getenv("SEEK") is not None:
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(os.getenv("SEEK")))

  if W > 1024:
    downscale = 1024.0/W
    F *= downscale
    H = int(H * downscale)
    W = 1024
  print("using camera %dx%d with F %f" % (W,H,F))

  K = np.array([[F,0,W//2],[0,F,H//2],[0,0,1]])
  Kinv = np.linalg.inv(K)

  if os.getenv("HEADLESS") is None:
    disp2d = Display2D(W, H)

  slam = SLAM(W, H, K)

  """
  mapp.deserialize(open('map.json').read())
  while 1:
    disp3d.paint(mapp)
    time.sleep(1)
  """

  gt_pose = None
  if len(sys.argv) >= 3:
    gt_pose = np.load(sys.argv[2])['pose']
    gt_pose[:, :3, 3] *= 50

  i = 0
  stabilizer = Stabilizer()
  while cap.isOpened():
    ret, frame = cap.read()
    frame = cv2.resize(frame, (W, H))
    stab_frame = stabilizer.stabilize(frame)
    if type(frame) == type(stab_frame):
      frame = np.ascontiguousarray(stab_frame, dtype=np.uint8)

    print("\n*** frame %d/%d ***" % (i, CNT))
    if ret == True:
      slam.process_frame(frame, None if gt_pose is None else np.linalg.inv(gt_pose[i]))
    else:
      break

    # 3-D display
    if disp3d is not None:
      disp3d.paint(slam.mapp)

    if disp2d is not None:
      img = slam.mapp.frames[-1].annotate(frame)
      disp2d.paint(img)

    i += 1
    """
    if i == 10:
      with open('map.json', 'w') as f:
        f.write(mapp.serialize())
        exit(0)
    """

