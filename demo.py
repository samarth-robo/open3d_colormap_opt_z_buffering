import open3d
import os
import cv2
import numpy as np
osp = os.path

path = osp.expanduser(osp.join('~', 'Downloads', 'fountain', 'fountain_small'))
debug_mode = False


def get_file_list(dir, extension):
    _, _, filenames = os.walk(dir).next()
    ext = extension[1:]
    filenames = [fn for fn in filenames if fn.split('.')[-1] == ext]
    return sorted(filenames)


def save_depth_maps(mesh, intrinsic, extrinsics, filenames):
  glb = save_depth_maps
  glb.index = -1

  # intrinsics assumed by the renderer - corrected later
  cx = intrinsic.width/2.0-0.5
  cy = intrinsic.height/2.0-0.5
  f = max(intrinsic.get_focal_length())
  glb.intrinsic = open3d.PinholeCameraIntrinsic(intrinsic.width, intrinsic.height,
    f, f, cx, cy)
  glb.extrinsics = extrinsics
  glb.filenames = filenames
  glb.vis = open3d.Visualizer()

  # affine matrix for correction
  offset_x = intrinsic.get_principal_point()[0] - \
             intrinsic.get_focal_length()[0]/f*cx
  offset_y = intrinsic.get_principal_point()[1] - \
             intrinsic.get_focal_length()[1]/f*cy
  glb.affine_M = np.asarray([
    [intrinsic.get_focal_length()[0]/f, 0, offset_x],
    [0, intrinsic.get_focal_length()[1]/f, offset_y]],
    dtype=np.float32)

  def callback(vis):
    ctr = vis.get_view_control()
    glb = save_depth_maps
    if glb.index >= 0:
      depth = np.asarray(vis.capture_depth_float_buffer(False), dtype=np.float32)
      depth = cv2.warpAffine(depth, glb.affine_M,
        (depth.shape[1], depth.shape[0]), cv2.WARP_INVERSE_MAP,
        cv2.BORDER_CONSTANT, 0)
      cv2.imwrite(glb.filenames[glb.index], np.uint16(np.asarray(depth) * 1000))

    glb.index = glb.index + 1
    if glb.index < len(glb.extrinsics):
      ctr.convert_from_pinhole_camera_parameters(glb.intrinsic,
        glb.extrinsics[glb.index])
    else:
      glb.vis.register_animation_callback(None)
    return False

  vis = glb.vis
  vis.create_window(width=intrinsic.width, height=intrinsic.height)
  vis.add_geometry(mesh)
  vis.register_animation_callback(callback)
  vis.run()
  vis.destroy_window()


if __name__ == "__main__":
    open3d.set_verbosity_level(open3d.VerbosityLevel.Debug)

    # Read camera pose and mesh
    camera = open3d.read_pinhole_camera_trajectory(os.path.join(path, "scene/key.log"))
    mesh = open3d.read_triangle_mesh(os.path.join(path, "scene", "integrated.ply"))

    # Read RGBD images
    rgbd_images = []
    color_image_path = get_file_list(
            os.path.join(path, "image/"), extension = ".jpg")
    depth_image_path = [osp.join('/tmp', fn.replace('jpg', 'png'))
      for fn in color_image_path]
    assert(len(depth_image_path) == len(color_image_path))

    # generate depth maps
    save_depth_maps(mesh, camera.intrinsic, np.asarray(camera.extrinsic),
      depth_image_path)

    for i in range(len(depth_image_path)):
        depth = open3d.read_image(depth_image_path[i])

        # add artificial noise to the depth maps
        depth = np.asarray(depth, dtype=np.float)
        depth += 30*np.random.rand(*depth.shape)
        depth = open3d.Image(depth.astype(np.uint16))

        color = open3d.read_image(os.path.join(path, 'image', color_image_path[i]))
        rgbd_image = open3d.create_rgbd_image_from_color_and_depth(color, depth,
                convert_rgb_to_intensity = False)
        if debug_mode:
            pcd = open3d.create_point_cloud_from_rgbd_image(rgbd_image,
                    open3d.PinholeCameraIntrinsic(open3d.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
            open3d.draw_geometries([pcd])
        rgbd_images.append(rgbd_image)

    # Before full optimization, let's just visualize texture map
    # with given geometry, RGBD images, and camera poses.
    option = open3d.ColorMapOptmizationOption()
    option.maximum_iteration = 0
    open3d.color_map_optimization(mesh, rgbd_images, camera, option)
    open3d.draw_geometries([mesh])
    open3d.write_triangle_mesh(os.path.join(path, "scene",
        "color_map_before_optimization.ply"), mesh)

    # # Optimize texture and save the mesh as texture_mapped.ply
    # # This is implementation of following paper
    # # Q.-Y. Zhou and V. Koltun,
    # # Color Map Optimization for 3D Reconstruction with Consumer Depth Cameras,
    # # SIGGRAPH 2014
    # option.maximum_iteration = 300
    # option.non_rigid_camera_coordinate = False
    # open3d.color_map_optimization(mesh, rgbd_images, camera, option)
    # open3d.draw_geometries([mesh])
    # open3d.write_triangle_mesh(os.path.join(path, "scene",
    #     "color_map_after_optimization.ply"), mesh)