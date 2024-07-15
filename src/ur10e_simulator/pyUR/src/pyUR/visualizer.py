import copy

import open3d as o3d
import os
import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering


class Visualizer:
    """Class to help with custom rendering"""

    def __init__(self, client=None):
        """

        :param client: The client to be used for the visualizer.
        :type client: int, optional, default=None
        """
        self.client = client
        self.meshes = {}

        self.meshes = {}
        self.R_urdf = {}

        # Init GUI, windows, etc.
        self.gui = gui.Application.instance
        self.gui.initialize()
        self.window = self.gui.create_window("pyUR", 640, 480, 0, 0)

        self.gui.menubar = gui.Menu()
        self.menu = self.gui.menubar
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)

        self.vis = self.scene.scene
        bc_color = [0, 1, 1, 1]
        self.scene.background_color = o3d.visualization.gui.Color(*bc_color)
        self.vis.set_background(bc_color)
        self.vis.set_lighting(self.vis.NO_SHADOWS, [0, 0, 0])

        # prepare default material
        self.mat = rendering.MaterialRecord()
        self.mat.shader = 'defaultLit'

        # Prepare the menu
        self.file_menu = gui.Menu()
        self.file_menu.add_item("RGB image", 0)
        self.file_menu.add_item("depth image", 1)
        menu_counter = 2
        self.menu.add_menu("Save", self.file_menu)

        for menu_id in range(menu_counter):
            c = self.MenuCallback(menu_id, self)
            self.window.set_on_menu_item_activated(menu_id, c)

        self.window.add_child(self.scene)

        self.file_dir = os.path.dirname(os.path.abspath(__file__))
        self.orientations = []
        self.positions = []
        self.colors = []
        self.paths = []

        # Run this, to load the initial meshes, compute bounding boxes and init camera and center of rotation
        self.read_info(self.client.robot)
        self.show_first()
        self.show_mesh()  # this is here only for bounding box computation

        # compute center of all objects
        scene_mesh = o3d.geometry.TriangleMesh()
        for f_path, m in self.meshes.items():
            R = self.vis.get_geometry_transform(f_path)
            m.transform(R)
            scene_mesh += m
            m.transform(np.linalg.inv(R))
        bbox = scene_mesh.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        # look at the center
        self.scene.look_at(center, center+[-1, 0, 0], [0, 0, 1])
        self.scene.center_of_rotation = center

        for obj_id, obj_name, _, _, _ in self.client.other_objects:
            self.read_info(obj_id)
            self.show_first(urdf_name=obj_name)
            self.show_mesh()
        self.is_alive = True

    def show_first(self, urdf_name="robot"):
        """
        Show the first batch of meshes in the visualizer. It loads the meshes and saves the to dict for quicker use later

        :param urdf_name: The name of the urdf to be used.
        :type urdf_name: str, optional, default="robot"
        """
        for mesh_id in range(len(self.positions) // 3):
            # get correct values for given mesh
            col = self.colors[mesh_id * 3:(mesh_id + 1) * 3]
            f_path = self.paths[mesh_id]
            self.meshes[f_path] = o3d.io.read_triangle_mesh(f_path)

            # Just for visualization
            if not self.meshes[f_path].has_triangle_normals():
                self.meshes[f_path].compute_triangle_normals()
            if not self.meshes[f_path].has_vertex_normals():
                self.meshes[f_path].compute_vertex_normals()

            self.meshes[f_path].paint_uniform_color(col[:3])

            # URDF rotations and translations
            init_xyz, init_rpy, scale, link_name = self.find_xyz_rpy(os.path.basename(f_path), urdf_name=urdf_name)

            R_urdf = np.eye(4)
            R_urdf[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(self.client.getQuaternionFromEuler(init_rpy)), (3, 3))
            R_urdf[:3, 3] = init_xyz

            self.R_urdf[f_path] = R_urdf

            self.meshes[f_path].scale(scale, self.meshes[f_path].get_center())
            self.meshes[f_path].translate(self.meshes[f_path].get_center()*scale, relative=False)

            # Add the mesh
            self.vis.add_geometry(f_path, geometry=self.meshes[f_path], material=self.mat)

            if self.client.config.skin.use and "airskin" in f_path:
                f_path_on = f_path.replace(".obj", "_on.obj")
                self.meshes[f_path_on] = copy.deepcopy(self.meshes[f_path])
                self.meshes[f_path_on].paint_uniform_color([1, 0, 0])
                self.vis.add_geometry(f_path_on, geometry=self.meshes[f_path_on], material=self.mat)
                self.vis.show_geometry(f_path_on, False)

    def show_mesh(self):
        """
        Function to parse info about meshes from PyBullet

        """
        for mesh_id in range(len(self.positions) // 3):
            # get correct values for given mesh
            pos = self.positions[mesh_id * 3:(mesh_id + 1) * 3]
            ori = self.orientations[mesh_id * 4:(mesh_id + 1) * 4]
            f_path = self.paths[mesh_id]

            R_urdf = self.R_urdf[f_path]

            # get ori and position as 4x4 transformation matrix
            R = np.eye(4)
            R[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(ori), (3, 3))
            R[:3, 3] = pos

            self.vis.set_geometry_transform(f_path, R @ R_urdf)
            if self.client.config.skin.use and "airskin" in f_path:
                self.vis.set_geometry_transform(f_path.replace(".obj", "_on.obj"), R @ R_urdf)

        if self.client.config.skin.use:

            for f_path, on in self.client.activated_skin.items():
                self.vis.show_geometry(f_path, not on)
                self.vis.show_geometry(f_path.replace(".obj", "_on.obj"), on)

    def read_info(self, obj_id):
        """
        Read info from PyBullet

        :param obj_id: id of the object; given by pybullet
        :type obj_id: int
        :return: 0 for success
        :rtype: int
        """
        # All meshes from the current object
        visualData = self.client.getVisualShapeData(obj_id)
        self.positions = []
        self.orientations = []
        self.colors = []
        self.paths = []

        for m in visualData:
            # Get information about individual parts of the object
            f_path = m[self.client.visualShapeData["FILE"]].decode("utf-8")
            if "invisible" in f_path:
                continue
            if self.client.config.show_collision:
                f_path = f_path.replace("visual", "collision").replace(".obj", "_vhacd.obj")
            if f_path == "":
                continue

            col = m[self.client.visualShapeData["COLOR"]]
            link = m[self.client.visualShapeData["LINK"]]

            # non-base links
            if link != -1:
                # get link info
                linkState = self.client.getLinkState(obj_id, link, computeLinkVelocity=0,
                                                     computeForwardKinematics=0)
                # get orientation and position wrt URDF - better than in world
                ori = linkState[self.client.linkInfo["URDFORI"]]
                pos = linkState[self.client.linkInfo["URDFPOS"]]
            # link == -1 is base. For that, getBasePosition... needs to be used. This joint must not contain URDF visual xyz and rpy
            else:
                pos, ori = self.client.getBasePositionAndOrientation(obj_id)

            self.positions += pos
            self.orientations += ori
            self.colors += col[:-1]
            self.paths.append(f_path)

        return 0

    def render(self):
        """
        Render all the things

        """
        # read info
        self.read_info(self.client.robot)
        # add new
        self.show_mesh()

        for obj_id, _, fixed, _, _ in self.client.other_objects:
            if not fixed:
                self.read_info(obj_id)
                self.show_mesh()

        self.window.post_redraw()
        if not self.gui.run_one_tick():
            self.is_alive = False
            self.gui.quit()

    class MenuCallback:
        """
        Class to handle menu callbacks.
        """
        def __init__(self, menu_id, parent):
            """
            Initialize the MenuCallback class.

            :param menu_id: The id of the menu.
            :type menu_id: int
            :param parent: The parent class (Visualizer).
            :type parent: pointer to the class of visualizer.Visualizer type
            """
            self.id = menu_id
            self.parent = parent
            self.path = None
            self.dialog_opened = False

        def __call__(self):
            """
            What happens when the menu is clicked

            """

            if self.id > 5:
                self.parent.menu.set_checked(self.id, not self.parent.menu.is_checked(self.id))
            if self.id == 0:
                self.parent.vis.scene.render_to_image(lambda im: self.save_image(im, 0))
            elif self.id == 1:
                self.parent.vis.scene.render_to_depth_image(lambda im: self.save_image(im, 1))

        def save_image(self, im, mode):
            """
            Save the image. It shows FileDialog to find path for image save. It saves it with the current resolution
            of the window.

            :param im: The image to be saved.
            :type im: open3d.geometry.Image
            :param mode: The mode of the image. 0 for RGB, 1 for depth.
            :type mode: int
            """
            dia = gui.Dialog("path_save_dialog")
            fd = gui.FileDialog(gui.FileDialog.Mode.SAVE, "Where to save?", self.parent.window.theme)
            fd.set_on_done(self.input_completed)
            fd.set_on_cancel(self.input_completed)
            fd.set_path(os.path.normpath(os.path.join(self.parent.client.file_dir, "../../..")))
            dia.add_child(fd)
            self.dialog_opened = True
            self.parent.window.show_dialog(dia)

            self.wait_for_dialog_completion()

            self.parent.window.close_dialog()

            if self.path is None:
                return 0
            if mode == 1:
                im = o3d.geometry.Image((255*np.asarray(im)).astype(np.uint8))
            o3d.io.write_image(self.path, im)

            self.parent.client.logger.info(f"File saved to {self.path}")
            self.path = None

        def wait_for_dialog_completion(self):
            """
            Help function to keep the gui loop running

            """
            while self.dialog_opened:
                self.parent.window.post_redraw()
                self.parent.gui.run_one_tick()

        def input_completed(self, text=None):
            self.path = text
            self.dialog_opened = False

    def find_xyz_rpy(self, mesh_name, urdf_name="robot"):
        """
        Find the xyz, rpy and scales values.

        :param mesh_name: The name of the mesh.
        :type mesh_name: str
        :param urdf_name: The name of the urdf.
        :type urdf_name: str, optional, default="robot"
        :return: The xyz, rpy, and scales, link_name
        """
        if self.client.config.show_collision:
            mesh_name = mesh_name.replace("_vhacd.obj", ".obj")
        for link in self.client.urdfs[urdf_name].links:
            if hasattr(link, "visual"):
                for idx in range(len(link.visual.geometry.mesh.filename)):
                    if os.path.basename(link.visual.geometry.mesh.filename) == mesh_name:
                        xyz = link.visual.origin.xyz
                        rpy = link.visual.origin.rpy
                        if hasattr(link.visual.geometry.mesh, "scale"):
                            scale = link.visual.geometry.mesh.scale[0]
                        else:
                            scale = 1
                        link_name = link.name
                        return xyz, rpy, scale, link_name
