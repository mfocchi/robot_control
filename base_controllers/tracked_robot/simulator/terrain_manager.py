# import open3d as o3d
# # Load mesh and convert to open3d.t.geometry.TriangleMesh
# # Load mesh and convert to open3d.t.geometry.TriangleMesh
# cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
# cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
#
# scene = o3d.t.geometry.RaycastingScene()
# cube_id = scene.add_triangles(cube)
#
# print(cube_id)
#
#
# # We create two rays:
# # The first ray starts at (0.5,0.5,10) and has direction (0,0,-1).
# # The second ray start at (-1,-1,-1) and has direction (0,0,-1).
# rays = o3d.core.Tensor([[0.5, 0.5, 10, 0, 0, -1], [-1, -1, -1, 0, 0, -1]],
#                        dtype=o3d.core.Dtype.Float32)
#
# ans = scene.cast_rays(rays)
#
# print(ans.keys())


import numpy as np
import open3d as o3d
import math
import rospkg

class TerrainManager:
    def __init__(self, mesh_path = "terrain.stl" ):
        self.mesh =  o3d.io.read_triangle_mesh(mesh_path)
        self.triangle_mesh = o3d.t.geometry.TriangleMesh.from_legacy(self.mesh)
        self.baseline = 10

    def project_on_mesh(self, point, direction, debug=False):
        # SUGGESTED BY https://github.com/matteodv99tn
        # define scene
        scene = o3d.t.geometry.RaycastingScene()
        # returns the ID for the added geometry
        scene.add_triangles(self.triangle_mesh)

        point = np.append(point, -self.baseline)
        # use open3d ray casting to compute distance from point to surface
        ray = o3d.core.Tensor([np.concatenate((point, direction))], dtype=o3d.core.Dtype.Float32)

        #The result contains information about a possible intersection with the geometry in the scene.
        ans = scene.cast_rays(ray)

        # self.logger.info(f"Distance from pelvis joint to surface: {ans}")
        # self.logger.info(f"Vertex ID: {ans['primitive_ids']}")

        #t_hit is the distance to the intersection. The unit is defined by the length of the ray direction. If there is no intersection this is inf
        distance = ans['t_hit'][0].cpu().numpy()
        z_coord = distance*direction
        eval_point = point + z_coord
        normal = ans['primitive_normals'][0].cpu().numpy()

        #compute roll/pitch
        pitch = math.atan2(normal[0], normal[2])
        roll = math.atan2(-normal[1]*np.sin(pitch), normal[0])
        #to have it in the range between  -pi/2 and pi/2
        if roll > np.pi/2:
            roll-=np.pi
        if roll <-np.pi / 2:
            roll += np.pi

        # Visualize
        if debug:
            mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0, 0, 0])
            line_ray = self.draw_line(point, direction, self.baseline*2, colors=[[1,0,0]])
            line_normal = self.draw_line(eval_point, normal,5., colors=[[0,1,0]])
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.2).translate(eval_point)
            sphere.paint_uniform_color([1, 0, 0])
            # draw_geometries is high level function - it create a window and add geometries. If you put draw_geometries inside the lop, this will make many windows simultaneously and may cause problem.
            #o3d.visualization.draw_geometries([self.mesh, line_ray, line_normal,mesh_frame, sphere])
            self.visualize([self.mesh, line_ray, line_normal, mesh_frame, sphere])
            print("normal is: ", normal)
            print("eval point is ",eval_point)

        return eval_point, roll, pitch

    def draw_line(self, start, length, direction, colors =  [[1,0,0]]):
        end = start + length * direction
        points = [start, end]
        lines = [[0, 1]]
        line = o3d.geometry.LineSet()
        line.points = o3d.utility.Vector3dVector(points)
        line.lines = o3d.utility.Vector2iVector(lines)
        line.colors = o3d.utility.Vector3dVector(colors)
        return line

    def visualize(self, meshes):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        #only in 0.17
        # vis.get_render_option().line_width = 5
        # vis.get_render_option().point_size = 20
        for mesh in meshes:
            vis.add_geometry(mesh)
        vis.run()
        vis.destroy_window()

        # # Create an OffscreenRenderer
        # renderer = o3d.visualization.rendering.OffscreenRenderer(800, 600)
        # # Access the scene
        # scene = renderer.scene
        # # Set up camera
        # camera = scene.camera
        # camera.look_at([0, 0, 0],  # Center of the scene
        #                eye=[3, 3, 3],  # Camera position
        #                up=[0, 1, 0])  # Up direction
        # # Add a directional light to cast shadows and create shading
        # scene.add_directional_light(
        #     direction=[-1, -1, -1],  # Direction of the light source
        #     color=[1.0, 1.0, 1.0],  # White light
        #     intensity=2.0,  # Intensity of the light
        #     cast_shadows=True  # Enable shadows
        # )
        # # Add ambient light (optional) to brighten up the scene slightly
        # scene.set_ambient_light([0.1, 0.1, 0.1])  # Soft ambient light for overall illumination
        # # Render the scene to an image
        # image = renderer.render_to_image()
        # # Save or visualize the rendered image
        # o3d.io.write_image("shaded_mesh.png", image)

    #non blocking
    # def visualize(self, mesh):
    #     o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    #     vis = o3d.visualization.Visualizer()
    #     vis.create_window()
    #     vis.update_geometry(mesh)
    #     vis.poll_events()
    #     vis.update_renderer()
    #     vis.destroy_window()
    #     return

if __name__ == '__main__':

    scaling_factor = 1
    point = np.array([22., 0.])
    direction = np.array([0., 0., 1.])

    mesh_path = rospkg.RosPack().get_path('tractor_description') + "/meshes/terrain.stl"
    terrainManager = TerrainManager(mesh_path)

    # mesh = o3d.io.read_triangle_mesh(mesh_path)
    # terrainManager.visualize(mesh)
    #if you want to rotate the mesh
    # R = mesh.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0 ))
    # mesh.rotate(R, center=(0, 0, 0))#rotate pi/2 roll
    # scale uniformly
    # mesh.scale(0.001, center=(0, 0, 0))
    #scale not uniformly
    #mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) * np.array([0.5, 0.5, 0.2]))

    eval_point, roll, pitch = terrainManager.project_on_mesh(point=point, direction=direction, debug=True)

    # print(scaling_factor * np.array([1, 1, 0.5]))
    print(eval_point)
    print(roll)
    print(pitch)



