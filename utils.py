import numpy as np
import open3d as o3d
import math
from math import isclose
import copy
import random

def ReadPcdPoint(f_number):
    """ read point from ply

    Args:
        fname (str): path to ply file

    Returns:
        [ndarray]: N x 3 point clouds
    """
    point_cloud_optimized = []
    path = "/home/holon-01/Desktop/ground_detection/new_dataset/5fps/%03d_base.pcd"%f_number
    pcd = o3d.io.read_point_cloud(path)
    print("la taille du pc",pcd)
    
    pcd.voxel_down_sample(0.002)
    pcd_down, _ = pcd.remove_statistical_outlier( nb_neighbors=20, std_ratio=2.0)
    point_cloud = np.asarray(pcd_down.points)
    for elemenet in point_cloud :
        # orbe de 2.2 m  
        #compute the distance between each point in point_cloud and the orgin
        distance = 2.0
        norm_vect_plane = math.sqrt(elemenet[0]**2+elemenet[1]**2 )
        #condition to define our trust function
        if norm_vect_plane <= distance :
            point_cloud_optimized.append(elemenet)
    #reshape the point cloud 
    point_cloud_optimized = np.reshape(point_cloud_optimized, (-1,3))

    return point_cloud_optimized, point_cloud


def PlaneRegression(points, threshold=0.01, init_n=3, iter=1000):
    """ plane regression using ransac

    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.

    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """
    pcd = NumpyToPCD(points)
    w, index = pcd.segment_plane(
        threshold, init_n, iter)

    return w, index




def DetectMultiPlanes(points, min_ratio=0.05, threshold=0.01, iterations=10000):
    """ Detect multiple planes from given point clouds

    Args:
        points (np.ndarray): 
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.

    Returns:
        [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
    """

    plane_list = []
    
    N = len(points)
    target = copy.deepcopy(points)
    
    count = 0

    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold=threshold, init_n=3, iter=iterations)
    
        count += len(index)
        plane_list.append((w, target[index]))
        target = np.delete(target, index, axis=0)

    return plane_list



    



def DrawResult(points, colors):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
    o3d.visualization.draw_geometries([pcd,axes])

def rotate_x(data,angle):

	
	#rotation 180° sur l'axe de x
	#angle = alpha /180 *pi
	flip_transform = [ [1, 0, 0, 0],  
			  [0, np.cos(angle), -np.sin(angle), 0],
			  [0, np.sin(angle), np.cos(angle), 0],
			  [0, 0, 0, 1]]
	data.transform(flip_transform) 
	return data

def rotate_y(data,angle):

	
	#rotation 180° sur l'axe de y
	#angle = alpha /180 *pi
	
	flip_transform = [ [np.cos(angle), 0, np.sin(angle), 0],  
			  [0, 1, 0, 0],
			  [-np.sin(angle), 0, np.cos(angle), 0],
			  [0, 0, 0, 1]] 

	data.transform(flip_transform) 
	return data
    
def rotate_z(data,angle):
	data = NumpyToPCD(data)
	#rotation 180° sur l'axe de z
	#angle = alpha /180 *pi
	
	flip_transform = [ [np.cos(angle), np.sin(angle), 0, 0],  
			  [-np.sin(angle), np.cos(angle), 0, 0],
			  [0, 0, 1, 0],
			  [0, 0, 0, 1]]
	
	data.transform(flip_transform)
	return np.asarray(data.points)

def estim_normal(points):
    pcd = NumpyToPCD(points)
	#normal estimation 
    pcd.estimate_normals(
	         search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=200))
    normal = np.asarray(pcd.normals)
    for item in normal:
            item[2]= np.abs(item[2])
            normal[:][2] = item[2]
            norm = np.mean(normal, axis=0)
    length_norm = np.linalg.norm(norm,axis=0)
    fact = 1/length_norm
    norm = norm * fact
    length_norm =  np.linalg.norm(norm,axis=0)

    return norm


def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud 

    Args:
        xyz (ndarray): 

    Returns:
        [open3d.geometry.PointCloud]: 
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd


def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray

    Args:
        pcd (open3d.geometry.PointCloud): 

    Returns:
        [ndarray]: 
    """

    return np.asarray(pcd.points)

