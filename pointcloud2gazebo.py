import copy

import numpy as np
import open3d as o3d

from pointcloud2gazebo_config import PointCloud2GazeboConfig


class PointCloud2Gazebo:
    """Create Gazebo World from 3D point cloud
    """
    def __init__(self, file_name: str):
        self.config = PointCloud2GazeboConfig()
        self.file_name = file_name
        self.pcd = o3d.io.read_point_cloud(file_name)
        self.mesh = None
        self.mesh_path = "models/pointcloud2gazebo/meshes/mesh.stl"

    def generate_gazebo_world(self):
        """Function to generate Gazebo World
        """
        # pre processing
        offset_pcd = self.__pre_process()

        # generate mesh
        self.mesh = self.__generate_mesh(offset_pcd)

        # write STL
        o3d.io.write_triangle_mesh(self.mesh_path, self.mesh)

    def visualization(self):
        """Function to visualize mesh
        """
        line_grid = self.__generate_grid(pitch=1, length=15)
        coord_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        o3d.visualization.draw_geometries([self.mesh, coord_mesh, line_grid])

    def __crop_pointcloud(self, pcd: o3d.geometry.PointCloud, min_z: float, max_z: float) -> o3d.geometry.PointCloud:
        """Function to crop pointcloud

        Args:
            pcd (o3d.geometry.PointCloud): Input pointcloud
            min_z (float): Minimum height(meter) of points to make Gazebo world.
            max_z (float): Maximum height(meter) of points to make Gazebo world.

        Returns:
            o3d.geometry.PointCloud: Cropped pointcloud
        """
        bbox = pcd.get_axis_aligned_bounding_box()
        min_bound = bbox.get_min_bound()
        max_bound = bbox.get_max_bound()

        # overwrite with min_z and max_z
        min_bound[2] = min_z
        max_bound[2] = max_z

        # create bounding box
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound.reshape(3, 1),
            max_bound.reshape(3, 1),
        )
        cropped_pcd = pcd.crop(bbox)
        return cropped_pcd

    def __noise_removal(self, pcd: o3d.geometry.PointCloud, nb_points: int, radius: float) -> o3d.geometry.PointCloud:
        """Function to remove noise from pointcloud

        Args:
            pcd (o3d.geometry.PointCloud): Input pointcloud
            nb_points (int): Number of points within the radius.
            radius (float): Radius of the sphere.

        Returns:
            o3d.geometry.PointCloud: Filtered pointcloud
        """
        _, index = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
        inlier_pcd = pcd.select_by_index(index)
        return inlier_pcd

    def __pre_process(self) -> o3d.geometry.PointCloud:
        """Function to do pre-processing

        Returns:
            o3d.geometry.PointCloud: Preprocessed pointcloud
        """
        # down sampling
        voxel_down_pcd = self.pcd.voxel_down_sample(voxel_size=self.config.voxel_down_sample.voxel_size)

        # cropping
        cropped_pcd = self.__crop_pointcloud(voxel_down_pcd,
                                             min_z=self.config.crop.min_z,
                                             max_z=self.config.crop.max_z)

        # noise removal
        inlier_pcd = self.__noise_removal(cropped_pcd,
                                          self.config.noise_removal.nb_points,
                                          self.config.noise_removal.radius)

        # apply offset
        offset_pcd = copy.deepcopy(inlier_pcd).translate((0, 0, -self.config.crop.min_z))
        return offset_pcd

    def __generate_mesh(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.TriangleMesh:
        """Function to generate mesh from pointcloud

        Args:
            pcd (o3d.geometry.PointCloud): Input pointcloud

        Returns:
            o3d.geometry.TriangleMesh: TriangleMesh
        """
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, self.config.triangle_mesh.alpha)
        mesh.compute_vertex_normals()
        return mesh

    def __generate_grid(self, pitch: float, length: int) -> o3d.geometry.LineSet:
        """Function to generate grid for visualization

        Args:
            pitch (float): pitch of grid
            length (int): length of grid

        Returns:
            o3d.geometry.LineSet: grid
        """
        line_set = o3d.geometry.LineSet()
        max_value = length * pitch
        x = np.arange(-max_value, max_value+pitch, pitch)
        x = np.repeat(x, 2)
        y = np.full_like(x, -max_value)
        y[::2] = max_value
        z = np.zeros_like(x)
        points_Y = np.vstack((x, y, z)).T

        y = np.arange(-max_value, max_value+pitch, pitch)
        y = np.repeat(y, 2)
        x = np.full_like(y, -max_value)
        x[::2] = max_value
        z = np.zeros_like(y)
        points_X = np.vstack((x, y, z)).T

        points = np.vstack((points_X, points_Y))
        line_set.points = o3d.utility.Vector3dVector(points)
        lines = np.arange(points.shape[0]).reshape(-1, 2)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color((0.5, 0.5, 0.5))

        return line_set
