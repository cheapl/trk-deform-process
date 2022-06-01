import open3d as o3d
import numpy as np
import copy


def draw_registration_result(ref_mesh, tar_mesh):
    ref_mesh = copy.deepcopy(ref_mesh)
    tar_mesh = copy.deepcopy(tar_mesh)
    ref_mesh.paint_uniform_color([1, 0.706, 0])
    tar_mesh.paint_uniform_color([0, 0.651, 0.929])

    ref_mesh.compute_vertex_normals()
    tar_mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([ref_mesh, tar_mesh])


def ref_to_tar_error(ref_mesh,tar_mesh):
    rec_pcd = o3d.geometry.PointCloud()
    rec_pcd.points = ref_mesh.vertices
    tar_pcd = o3d.geometry.PointCloud()
    tar_pcd.points = tar_mesh.vertices
    errors = rec_pcd.compute_point_cloud_distance(tar_pcd)
    np_errors = np.asarray(errors)

    #print(list(np_errors))

    error_num = len(np_errors)
    torr = 0
    for i in range(error_num):
        #if np_errors[i] < 0.0005:
        if np_errors[i] < 0.0012:
            torr = torr + 1
    error_rate = torr / error_num
    error_sum = np.sum(np_errors)
    return error_sum, error_rate


def read_triangle_mesh_obj(fname):
    vertices_list = []
    triangles_list = []
    mesh_obj = open(fname, "r")
    mesh_obj_line = mesh_obj.readline()
    while mesh_obj_line:
        mesh_obj_line_list = mesh_obj_line.split()
        #print(mesh_obj_line_list)
        if (mesh_obj_line_list[0] == 'v'):
            vertex = []
            vertex.append(float(mesh_obj_line_list[1]))
            vertex.append(float(mesh_obj_line_list[2]))
            vertex.append(float(mesh_obj_line_list[3]))
            vertices_list.append(vertex)
        elif (mesh_obj_line_list[0] == 'f'):
            face = []
            face.append(int(mesh_obj_line_list[1]))
            face.append(int(mesh_obj_line_list[2]))
            face.append(int(mesh_obj_line_list[3]))
            triangles_list.append(face)
        mesh_obj_line = mesh_obj.readline()
    mesh_obj.close()

    np_vertices = np.asarray(vertices_list)
    np_triangles = np.asarray(triangles_list)

    rec_mesh = o3d.geometry.TriangleMesh()
    # Copy vertices, normals and faces of graph to this new mesh
    rec_mesh.vertices = o3d.utility.Vector3dVector(np_vertices)
    rec_mesh.triangles = o3d.utility.Vector3iVector(np_triangles)

    return rec_mesh


def corr_dist(ref_mesh, tar_mesh):
    '''
    source = np.asarray(ref_mesh.vertices)
    target = np.asarray(tar_mesh.vertices)
    source_num = len(source)
    target_num = len(target)
    corr_dists = []
    if source_num != target_num:
        print("Please choose correspodent meshes!")
    for i in range(source_num):
        source_pos = source[i]
        target_pos = target[i]
        dist = np.linalg.norm(source_pos - target_pos)
        corr_dists.append(dist)

    np_corr_dists = np.asarray(corr_dists)
    print(np_corr_dists)
    '''
    source = o3d.geometry.PointCloud()
    source.points = ref_mesh.vertices
    target = o3d.geometry.PointCloud()
    target.points = tar_mesh.vertices
    errors = source.compute_point_cloud_distance(target)
    np_errors = np.asarray(errors)
    error_mean = np.mean(np_errors)

    print(error_mean)


    f = open("out/rigid_vertices.txt", "a")
    cout = 0
    for j in range(len(source.points)):
        if np_errors[j] < error_mean/2:
            f.write(str(j)+"\n")
            cout += 1;
    rate = cout / len(source.points)
    print(rate)
    f.close()
