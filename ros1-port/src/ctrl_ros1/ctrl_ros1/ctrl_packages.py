"""catkin package share lookup, including devel and install workspaces."""
import rospkg

PackageNotFoundError = rospkg.ResourceNotFound


def get_package_share_directory(package_name):
    return rospkg.RosPack().get_path(package_name)
