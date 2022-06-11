import open3d as o3d
import numpy as np


class DoorHandleDetect:
	def ReadPlyPoint(self, fname):
		""" read point from ply

		Args:
			fname (str): path to ply file

		Returns:
			[ndarray]: N x 3 point clouds
		"""

		pcd = o3d.io.read_point_cloud(fname)

		return self.PCDToNumpy(pcd)


	def NumpyToPCD(self, xyz):
		""" convert numpy ndarray to open3D point cloud 

		Args:
			xyz (ndarray): 

		Returns:
			[open3d.geometry.PointCloud]: 
		"""

		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(xyz)

		return pcd


	def PCDToNumpy(self, pcd):
		"""  convert open3D point cloud to numpy ndarray

		Args:
			pcd (open3d.geometry.PointCloud): 

		Returns:
			[ndarray]: 
		"""

		return np.asarray(pcd.points)


	def RemoveNan(self, points):
		""" remove nan value of point clouds

		Args:
			points (ndarray): N x 3 point clouds

		Returns:
			[ndarray]: N x 3 point clouds
		"""

		return points[~np.isnan(points[:, 0])]


	def RemoveNoiseStatistical(self, pc, nb_neighbors=20, std_ratio=2.0):
		""" remove point clouds noise using statitical noise removal method

		Args:
			pc (ndarray): N x 3 point clouds
			nb_neighbors (int, optional): Defaults to 20.
			std_ratio (float, optional): Defaults to 2.0.

		Returns:
			[ndarray]: N x 3 point clouds
		"""

		pcd = self.NumpyToPCD(pc)
		cl, ind = pcd.remove_statistical_outlier(
			nb_neighbors=nb_neighbors, std_ratio=std_ratio)

		return self.PCDToNumpy(cl)


	def DownSample(self, pts, voxel_size=0.003):
		""" down sample the point clouds

		Args:
			pts (ndarray): N x 3 input point clouds
			voxel_size (float, optional): voxel size. Defaults to 0.003.

		Returns:
			[ndarray]: 
		"""

		p = self.NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

		return self.PCDToNumpy(p)


	def PlaneRegression(self, points, threshold=0.01, init_n=3, iter=1000):
		""" plane regression using ransac

		Args:
			points (ndarray): N x3 point clouds
			threshold (float, optional): distance threshold. Defaults to 0.003.
			init_n (int, optional): Number of initial points to be considered inliers in each iteration
			iter (int, optional): number of iteration. Defaults to 1000.

		Returns:
			[ndarray, List]: 4 x 1 plane equation weights, List of plane point index
		"""

		pcd = self.NumpyToPCD(points)

		w, index = pcd.segment_plane(
			threshold, init_n, iter)
		print("PlaneRegression")

		return w, index


	def DrawResult(self, points, colors, medians, m_colors):
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(points)
		pcd.colors = o3d.utility.Vector3dVector(colors)
		pcd_center = o3d.geometry.PointCloud()
		print("type points, colors, medians, mcolors", type(points), type(colors), type(np.array(medians)), type(np.array(m_colors)))
		print("medians" , medians)
		centers = np.array(medians)
		center_colors = np.array(m_colors)
		print("points", points, points.shape)
		print("pcd center", pcd.get_center())
		 
		print("centers shape", centers, centers.shape, len(centers))
		print("center_colors shape", center_colors, center_colors.shape, len(center_colors))
		pcd_center.points = o3d.utility.Vector3dVector(centers)
		#do some filtering here to only get the door handle center and none of the other centers
		#probably by its current y location? :)
		#julia please
		pcd_center.colors = o3d.utility.Vector3dVector(center_colors)
		
		o3d.visualization.draw_geometries([pcd, pcd_center])
	
	def FindDoorPlane(self, y_upper_threshold = 0.05, y_lower_threshold =0.05, points):
		#y_upper and y_lower threshold should be based on the inital position that the robot arm is sent to for this task.
		#which is something that still needs to be decided :')
		points = np.asarray(pcd.points)
		mask = points[:,1] > y_upper_threshold
		pcd.points = o3d.utility.Vector3dVector(points[mask]) # normals and colors are unchanged
		mask = points[:,1] > y_lower_threshold

		# alternative
		pcd = pcd.select_by_index(np.where(points[:,1] > y_theshold)[0])
	
	def DetectMultiPlanes(self, points, min_ratio=0.05, threshold=0.01, iterations=1000):
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
		target = points.copy()
		count = 0
		largest_plane = 0
		height_of_doorhandle = 12
		#find max x and y, only allow point clouds that have x and ys that are between that. This sounds way more complicated.
		z_of_largest_plane = 12

		while count < (1 - min_ratio) * N:
			#print("N and count is ", N, " ", count)
			w, index = self.PlaneRegression(
				target, threshold=.015, init_n=3, iter=iterations)
		
			count += len(index)
			num_points = len(target[index])
			if(num_points > largest_plane and target[index][0][2] < z_of_largest_plane): #if it is a larger plane, then append it! :')
				print("z of largest plane CHANGED. it is", z_of_largest_plane)
				largest_plane = num_points
				z_of_largest_plane =   target[index][0][2]
				plane_list.append((w, target[index]))
				target = np.delete(target, index, axis=0)
				continue
			print("abs(target[index][0][2] - z_of_largest_plane)", abs(target[index][0][2] - z_of_largest_plane))
			if(abs(target[index][0][2] - z_of_largest_plane) < height_of_doorhandle):
				plane_list.append((w, target[index]))           
				print("w is", w)
			target = np.delete(target, index, axis=0)
		#print("N is ", N)
		return plane_list