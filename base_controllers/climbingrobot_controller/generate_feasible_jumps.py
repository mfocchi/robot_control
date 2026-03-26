from base_controllers.utils.feasibility_graph import make_uniform_grid_yz, build_directed_jump_graph, save_edges_to_csv
from climbingrobot_controller2_light import  ClimbingrobotController
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
import matlab.engine
robotName = "climbingrobot2"
from base_controllers.utils.common_functions import getRobotModel, checkRosMaster
import base_controllers.params as conf
import numpy as np
import rospkg

def initOptimWithRealTerrain(p0, pf):
    #eval terrain
    p0[0] = p.terrainManager.wall_surface_eval(p0[2], p0[1], p.mesh_x, p.mesh_y, p.mesh_z) + 0.2
    pf[0] = p.terrainManager.wall_surface_eval(pf[2], pf[1], p.mesh_x, p.mesh_y, p.mesh_z) + 0.2
    converged = p.initOptim(p0,pf)
    if converged:
        return True
    elif np.linalg.norm(self.targetPosIdeal-self.targetPos) < 0.5:
        return True
    else:
        return False

if __name__ == '__main__':
    p = ClimbingrobotController(robotName)
    checkRosMaster()

    # this is for the matlab optim
    p.eng = matlab.engine.start_matlab()
    p.eng.addpath('./codegen_mesh', nargout=0)
    p.terrainManager = TerrainManager(grid_size=100, wall_depth=1, max_ridge_depth=0.5, seed="default", Lz=-20, Ly=5, generate_terrain=True, terrain_type='rock')
    p.mesh_x, p.mesh_y, p.mesh_z = p.terrainManager.get_mesh()

    p.robot = getRobotModel(p.robot_name, generate_urdf=True, xacro_path=rospkg.RosPack().get_path('climbingrobot_description') + '/urdf/' + p.robot_name + '.xacro')
    p.initVars()
    p.updateKinematicsDynamics()

    #test
    # p0 = np.array([0.69216,  2.58373, -6.17161])
    # pf =  np.array([0.19306,   1.42311, -12.01592])
    # p.initOptim(p0, pf)
    #
    # p0 = np.array([0.17693,   1.35084, -12.07002])
    # pf =  np.array([1.46107 ,  3.1  ,   -17.22812])
    # p.initOptim(p0, pf)

    pts = make_uniform_grid_yz(y_min=0.5, y_max=4.5, z_min=-19.0, z_max=1., ny=5, nz=20)
    print("Number of grid points:", pts.shape[0])
    #possibility to resume if you kill
    edges = build_directed_jump_graph(pts, is_feasible=initOptimWithRealTerrain, csv_path="loose_feasible_jumps.csv")

    #old save only at the end
    #save_edges_to_csv("feasible_jumps.csv", pts, edges)



        
