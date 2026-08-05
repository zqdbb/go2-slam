#!/usr/bin/env python3

import argparse
import os
import signal
import sys
import time

import numpy as np
import open3d as o3d
import trg_planner


class TRGRunner:

    def __init__(self, args):
        self.args = args
        root_dir = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        map_config_file = os.path.join(root_dir, 'config', args.map) + '.yaml'
        if not os.path.exists(map_config_file):
            raise FileNotFoundError(f'File {map_config_file} not found')

        self.sample_starts = []
        self.sample_goals = []
        if args.map == 'indoor':
            self.sample_starts = np.array([[3.27, 4.12, 0], [14.86, 50.61, 0],
                                           [34.16, -6.32, 0], [63.33, 5.11, 0],
                                           [48.03, 37.52, 0]])
            self.sample_goals = np.array([[17.18, 0.29, 0], [23.9, 40.4, 0],
                                          [43.00, -26.01,
                                           0], [69.60, 19.02, 0],
                                          [55.48, 54.97, 0]])
        elif args.map == 'mountain':
            self.sample_starts = np.array([[-7.22, -7.54,
                                            0], [-2.07, -2.21, 0],
                                           [13.04, -1.99,
                                            0], [17.96, 17.69, 0],
                                           [-6.56, 4.59, 0]])

            self.sample_goals = np.array([[-9.97, 3.56, 0], [7.52, 1.44, 0],
                                          [14.43, 6.87, 0], [9.49, 16.60, 0],
                                          [3.11, -6.68, 0]])

        self.trg_planner = trg_planner.TRGPlanner()
        self.trg_planner.setParams(map_config_file)
        self.trg_planner.init()

        signal.signal(signal.SIGINT, self.signal_handler)

        self.run()

    def planning(self, goal):
        self.trg_planner.setGoal(goal)
        while not self.trg_planner.getFlagPathFound():
            time.sleep(0.1)
        path = self.trg_planner.getPlannedPath()
        direct_dist, _, smth_length, planning_time, avg_risk = self.trg_planner.getPathInfo(
        )
        self.trg_planner.setFlagPathFound(False)
        return path, direct_dist, _, smth_length, planning_time, avg_risk

    def run(self):

        while True:
            if not self.trg_planner.getFlagGraphInit():
                time.sleep(0.1)
                continue

            if len(self.sample_starts) == 0 or len(self.sample_goals) == 0:
                break

            start = self.sample_starts[0]
            goal = self.sample_goals[0]
            self.sample_starts = self.sample_starts[1:]
            self.sample_goals = self.sample_goals[1:]
            self.trg_planner.setPose(pose=start)

            path, direct_dist, _, smth_length, planning_time, avg_risk = self.planning(
                goal)

            # print(f'Path found: {path}')
            print("==============================")
            print(f"Start: {start}")
            print(f"Goal: {goal}")
            print(f'Direct distance: {direct_dist}')
            print(f'Smoothed length: {smth_length}')
            print(f'Planning time: {planning_time}')
            print(f'Average risk: {avg_risk}')
            print("==============================")

            self.visualize(start, goal, path)
            # time.sleep(3)
            print("Now executing path...")

        print('All goals reached')
        self.trg_planner.shutdown()

    def visualize(self, start, goal, path):
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=800, height=600)

        # crop map
        preMap = self.trg_planner.getMapEigen("pre")
        min_point = np.min(path, axis=0)[:2]
        max_point = np.max(path, axis=0)[:2]
        margin = np.linalg.norm(max_point[:2] - min_point[:2]) * 0.5
        min_bound = min_point - margin
        max_bound = max_point + margin

        print(f"min_bound: {min_bound}, max_bound: {max_bound}")
        cropMap = preMap[(preMap[:, 0] > min_bound[0])
                         & (preMap[:, 0] < max_bound[0]) &
                         (preMap[:, 1] > min_bound[1]) &
                         (preMap[:, 1] < max_bound[1])]
        print(f"preMap: {preMap.shape}, cropMap: {cropMap.shape}")

        # Visualize pre map
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cropMap)
        pcd.colors = o3d.utility.Vector3dVector(
            np.tile([0.0, 0.0, 1.0], (len(cropMap), 1)))
        vis.add_geometry(pcd)

        # Visualize TRG nodes
        trg_ = self.trg_planner.getTRG()
        nodes = trg_.getGraphCopy("global")

        # check if node is in crop map
        crop_nodes = {}
        for node in nodes.values():
            if node.pos[0] < min_bound[0] or node.pos[0] > max_bound[
                    0] or node.pos[1] < min_bound[1] or node.pos[
                        1] > max_bound[1]:
                continue
            crop_nodes[node.id] = node

        for node in crop_nodes.values():
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            sphere.translate(node.pos + np.array([0, 0, 0.2]))
            sphere.paint_uniform_color([0, 1, 1])
            # vis.add_geometry(sphere)

        # Visualize edge
        for node in crop_nodes.values():
            for edge in node.edges:
                node2 = nodes[edge.dst_id]
                # check if node is in crop map
                if node.pos[0] < min_bound[0] or node.pos[0] > max_bound[
                        0] or node.pos[1] < min_bound[1] or node.pos[
                            1] > max_bound[1] or node2.pos[0] < min_bound[
                                0] or node2.pos[0] > max_bound[0] or node2.pos[
                                    1] < min_bound[1] or node2.pos[
                                        1] > max_bound[1]:
                    continue
                line = o3d.geometry.LineSet()
                n_pos1 = node.pos + np.array([0, 0, 0.2])
                n_pos2 = node2.pos + np.array([0, 0, 0.2])
                points = o3d.utility.Vector3dVector([n_pos1, n_pos2])
                lines = o3d.utility.Vector2iVector([[0, 1]])
                line.points = points
                line.lines = lines

                color = np.array([0, 1, 1],
                                 dtype=np.float64).reshape(-1, 3)  # cyan

                line.colors = o3d.utility.Vector3dVector(color)
                # vis.add_geometry(line)

        # Visualize path
        for i in range(len(path) - 1):
            path_point = path[i]
            next_point = path[i + 1]
            direction = (next_point - path_point) / np.linalg.norm(next_point -
                                                                   path_point)
            cylinder = o3d.geometry.TriangleMesh.create_cylinder(
                radius=0.1, height=np.linalg.norm(next_point - path_point))
            cylinder.rotate(
                o3d.geometry.get_rotation_matrix_from_axis_angle(
                    np.cross([0, 0, 1], direction)))
            cylinder.translate(path_point + (next_point - path_point) / 2 +
                               np.array([0, 0, 0.2]))

            cylinder.paint_uniform_color([1, 0, 0])

            vis.add_geometry(cylinder)

        opt = vis.get_render_option()
        opt.background_color = np.array([0, 0, 0])
        opt.point_size = 3.0
        opt.mesh_show_back_face = True

        ctr = vis.get_view_control()
        ctr.set_zoom(0.8)

        vis.run()
        vis.destroy_window()

    def signal_handler(self, sig, frame):
        print('Exiting and plotting data...')
        self.trg_planner.shutdown()
        sys.exit(0)


def get_args():
    parser = argparse.ArgumentParser(description='TRG-planner Python Runner')
    parser.add_argument('--map',
                        type=str,
                        default='mountain',
                        help='Configuration file to use')
    return parser.parse_args()


def main(args=None):
    try:
        args = get_args()
        _ = TRGRunner(args)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
