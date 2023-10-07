import os

import numpy as np
import open3d as o3d

DATA_DIR = os.path.join(os.path.abspath('../'), "Data", "sim", "meshes")


class ObjectVisualizer:

    def __init__(self):
        self.cur_mesh = None
        self.visualizer = o3d.visualization.Visualizer()
        self.visualizer.create_window()

        self.initialize_visualizer()

    def initialize_visualizer(self):
        render_opt = self.visualizer.get_render_option()
        render_opt.show_coordinate_frame = True
        render_opt.background_color = np.asarray([0.5, 0.5, 0.5])

    def update_visualizer(self):
        self.visualizer.poll_events()
        self.visualizer.update_renderer()

    def load_mesh_from_path(self, mesh_path):
        _, extension = os.path.splitext(mesh_path)
        if extension.lower() in ['.stl', '.obj']:
            self.cur_mesh = o3d.io.read_triangle_mesh(mesh_path, print_progress=True)

            self.cur_mesh.compute_vertex_normals()
            self.cur_mesh.paint_uniform_color([0.3, 0.3, 0.3])

            self.cur_mesh.scale(2, center=self.cur_mesh.get_center())

            # Get the vertices of the mesh
            vertices = np.asarray(self.cur_mesh.vertices)

            # Find the lowest Z-coordinate in the mesh
            lowest_z = np.min(vertices[:, 2])

            # Offset the mesh along the Z-axis
            self.cur_mesh.translate([0, 0, -lowest_z])

            self.disp_cur_mesh()
            return True

        print("[OBJ] Invalid file type!")
        return False

    def display_primitive(self, primitive, resolution, *args):
        if primitive == "cylinder" or primitive == 0:
            self.cur_mesh = o3d.geometry.TriangleMesh.create_cylinder(args[0], args[1], resolution)
        elif primitive == "sphere" or primitive == 1:
            self.cur_mesh = o3d.geometry.TriangleMesh.create_sphere(args[0], resolution)
        elif primitive == "rectangular prism" or primitive == 2:
            self.cur_mesh = o3d.geometry.TriangleMesh.create_box(args[0], args[1], args[2], resolution)
        else:
            print("[OBJ] Invalid primitive given!")
            return

        # clear visualizer to setup for new geometry
        self.clear_visualizer()

        # compute vertex normals for lighting
        self.cur_mesh.compute_vertex_normals()
        self.cur_mesh.paint_uniform_color([0.3, 0.3, 0.3])

        # find the centroid and average Z value to properly offset mesh
        centroid = self.cur_mesh.get_center()  # get the center of the mesh
        vertices = np.asarray(self.cur_mesh.vertices)
        # rectangular prisms are shifting by half their upper and lower verticies, while other prims are not.
        if primitive == "rectangular prism" or primitive == 2:
            z_offset = (vertices[:, 2].min() + vertices[:, 2].max()) / 2.0
        else:
            z_offset = -1 * vertices[:, 2].min()

        self.cur_mesh.translate(
            -centroid)  # move the object by the negative offset of the centroid, thereby centering it.
        self.cur_mesh.translate(
            np.array([0, 0, z_offset]))  # offset the object by the neg. of minimum z, thus lifting it off the ground.

    def pack_object(self, path=None):
        '''
            Writes 'self.curmesh' to disk for simulation usage.
        :param path: Path to export the .STL file to. (including filename)
        :return:
        '''
        if self.cur_mesh is None:
            print("[OBJ] Attempted to save a null object to disk! Aborting!")
            return

        if path is None:
            path = os.path.join(DATA_DIR, "object_exp.stl")

        o3d.io.write_triangle_mesh(path, self.cur_mesh)

    def disp_cur_mesh(self):
        self.clear_visualizer()
        self.visualizer.add_geometry(self.cur_mesh)

    def clear_visualizer(self):
        self.visualizer.clear_geometries()

    def group_by_faces(self):
        pass

    def group_by_z(self):
        pass
