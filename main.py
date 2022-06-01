# Libraries
import open3d as o3d
import numpy as np


import utility


if __name__ == '__main__':
    ref_mesh = o3d.io.read_triangle_mesh("data/dog_uni_100.obj")
    tar_mesh_GT = o3d.io.read_triangle_mesh("data/dog_uni_101.obj")
    tar_mesh = o3d.io.read_triangle_mesh("result/outres_dog_w.obj")

    #ref = utility.read_triangle_mesh_obj("data/dog_uni_100.obj")
    #tar = utility.read_triangle_mesh_obj("data/dog_uni_101.obj")
    #utility.corr_dist(ref,tar)

    e = utility.ref_to_tar_error(tar_mesh_GT, tar_mesh)
    print(e)
    utility.draw_registration_result(tar_mesh_GT, tar_mesh)