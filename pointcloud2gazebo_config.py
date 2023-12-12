from pathlib import Path

from traitlets import Float, Int
from traitlets.config.configurable import Configurable
from traitlets.config.loader import PyFileConfigLoader

CONFIG_FILE = "config/config.py"


class Crop(Configurable):
    """Configuration of Cropping
    """
    # Minimum and maximum height of points to make Gazebo world.
    min_z = Float(0.0).tag(config=True)
    max_z = Float(2.0).tag(config=True)

    def __init__(self, **kwargs):
        super(Crop, self).__init__(**kwargs)


class NoiseRemoval(Configurable):
    """Configuration of Noise Removal
    """
    # Number of points within the radius.
    nb_points = Int(16).tag(config=True)

    # Radius of the sphere.
    radius = Float(0.1).tag(config=True)

    def __init__(self, **kwargs):
        super(NoiseRemoval, self).__init__(**kwargs)


class VoxelDownSample(Configurable):
    """Configuration of Voxel DownSampling
    """
    # Voxel size(meter) to downsample into.
    voxel_size = Float(0.05).tag(config=True)

    def __init__(self, **kwargs):
        super(VoxelDownSample, self).__init__(**kwargs)


class TriangleMesh(Configurable):
    """Configuration of TriangleMesh
    """
    # Parameter to control the shape.
    # A very big value will give a shape close to the convex hull.
    alpha = Float(0.1).tag(config=True)

    def __init__(self, **kwargs):
        super(TriangleMesh, self).__init__(**kwargs)


class PointCloud2GazeboConfig(Configurable):
    """Configuration of PointCloud2Gazebo
    """
    def __init__(self, **kwargs):
        super(PointCloud2GazeboConfig, self).__init__(**kwargs)
        self.config = PyFileConfigLoader(str(Path(CONFIG_FILE))).load_config()
        self.crop = Crop(config=self.config)
        self.noise_removal = NoiseRemoval(config=self.config)
        self.voxel_down_sample = VoxelDownSample(config=self.config)
        self.triangle_mesh = TriangleMesh(config=self.config)
