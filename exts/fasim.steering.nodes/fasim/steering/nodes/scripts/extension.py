import omni.ext


class PublicExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # OmniGraph scans this extension for .ogn files when the module loads.
        return

    def on_shutdown(self):
        return
