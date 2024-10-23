import open3d
import numpy as np
def read_display_pcd_pc():
    path1 = "pcd_drawer_1.pcd"
    path2 = "pcd_drawer.pcd"
    pcd=open3d.io.read_point_cloud(path1)
    pcd2=open3d.io.read_point_cloud(path2)
    # 设置点云颜色 只能是0 1 如[1,0,0]代表红色为既r
    pcd.paint_uniform_color([0, 1, 0])
    #创建窗口对象
    vis=open3d.visualization.Visualizer()
    # 创建窗口，设置窗口标题
    vis.create_window(window_name="point_cloud")
    # 设置点云渲染参数
    opt=vis.get_render_option()
    # 设置背景色（这里为白色）
    opt.background_color=np.array([255, 255, 255])
    # 设置渲染点的大小
    opt.point_size=1.0
    # 添加点云
    vis.add_geometry(pcd)
    vis.add_geometry(pcd2)
    vis.run()

if __name__ =="__main__":
    read_display_pcd_pc()