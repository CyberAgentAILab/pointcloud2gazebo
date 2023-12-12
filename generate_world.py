import argparse

from pointcloud2gazebo import PointCloud2Gazebo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', help='Path to PoindCloud.', required=True)
    parser.add_argument('--vis', help='Visualize generated mesh', action='store_true')
    args = parser.parse_args()

    p2g = PointCloud2Gazebo(args.input)
    p2g.generate_gazebo_world()
    if args.vis:
        p2g.visualization()


if __name__ == "__main__":
    main()
