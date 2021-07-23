
from numpy import absolute, greater_equal, less_equal
from utils import *


back_to_line = "\n        \n"



def main():
    import random
<<<<<<< HEAD
    point_cloud,point_cloud_op = ReadPcdPoint(1)
=======
   
    point_cloud = ReadPcdPoint("path_to_data")
>>>>>>> b2dee180a21764f1a58160e8b8c89654e04fc485
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
    gravity = np.asarray([0,0,1])
    
    sol_o3d= o3d.geometry.PointCloud()
    planes = []
    colors = []
    centroids = []
    normals = []


    results= DetectMultiPlanes(point_cloud, min_ratio=0.05, threshold=0.005, iterations=2000)

    for _, plane in results:

        r = random.random()
        g = random.random()
        b = random.random()

        color = np.zeros((plane.shape[0], plane.shape[1]))
        color[:, 0] = r 
        color[:, 1] = g
        color[:, 2] = b

        centroid = np.mean(plane, axis=0 )
        normal  = estim_normal(plane)
    
        centroids.append(centroid)
        normals.append(normal)
        planes.append(plane)
        colors.append(color)
       
    print("normal ", len(normals))
    print("plans" , planes[0][0])

    
    rtol = 0.01
    normal_plan = []
    for j in range(0,len(normals)) :
        
        atol = np.abs(gravity-normals[j])    
        #^^^^ diffrence absolut tolerance between the gravity and each normal for a plan  
        rtol = np.abs(normals[j]) * rtol
        #^^^^ relative tolerance 
        math_close = np.maximum(rtol * np.maximum(np.abs(normals[j]), gravity),np.abs(rtol))
        #print("math_close:" ,math_close)
        
        if np.greater_equal(math_close,atol).any():
            print("index of ground is ", j)
            normal_plan.append(normals[j])
            print("value of each plan to catch the diff\n\n",planes[j])
            sol_ = np.asarray(planes[j])
            sol_o3d += NumpyToPCD(sol_)
    normal_plan_before_men = np.reshape(normal_plan, (-1,3))
    normal_plan = np.mean(normal_plan_before_men,axis =0 )
    #rotation x for 
    theta = np.arctan(normal_plan[0]/-normal_plan[2])
    print("angle of rotation between x and z ","         ",theta)
    
    #correct the ground towards  X
    sol_corrected_in_x = rotate_y(sol_o3d,theta)

    #rotation y for z
    phi = np.arctan((normal_plan[1]/-normal_plan[2]))
    print("angle of rotation between y and z ","         ",phi)
   
    #correct the ground towards  Y
    sol_corrected_in_x_and_y = rotate_x(sol_corrected_in_x,phi)
    
    planes = np.concatenate(planes, axis=0)
    colors = np.concatenate(colors, axis=0)


    DrawResult(planes, colors)
    o3d.visualization.draw_geometries([sol_o3d,axes])
    o3d.visualization.draw_geometries([sol_corrected_in_x,axes])
    o3d.visualization.draw_geometries([sol_corrected_in_x_and_y,axes])

if __name__ == "__main__":
    print("welcome to holon-code")
    main() 

