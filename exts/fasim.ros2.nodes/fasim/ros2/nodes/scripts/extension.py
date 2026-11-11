import omni.ext

from fasim.ros2.nodes.library_path import ensure_isaac_ros2_library_path


class PublicExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # OmniGraph scans this extension for .ogn files when the module loads.
        ensure_isaac_ros2_library_path()
        return

    def on_shutdown(self):
        return
