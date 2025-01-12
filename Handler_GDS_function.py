# -*- coding: utf-8 -*-
"""


简易生成GDS文件函数


"""
import gdstk
import phidl.geometry  as  pg
# ------------------------- 参数定义 -------------------------
# 定义布线和通孔的各种参数，包括线宽、线距、通孔尺寸等
VIA_LENGTH = 9        # 通孔长度
VIA_WIDTH = 3.8       # 通孔宽度

# 起点层线宽与线距
LINE_WIDTH_S = 2
LINE_SPACING_S = 2

# 终点层线宽与线距
LINE_WIDTH_T = 2
LINE_SPACING_T = 2

# 过滤距离计算
FILTER_DIS_S = LINE_WIDTH_S / 2 + LINE_SPACING_S
FILTER_DIS_T = LINE_WIDTH_T / 2 + LINE_SPACING_T

# 版图缩放因子
SCALE_FACTOR = 100
# ------------------------- 缩放函数 -------------------------
def scale_line_coordinates(lines, scale_factor):
    """
    缩放线段坐标。

    参数:
    - lines: 线段列表，每个线段为 ((x1, y1), (x2, y2))
    - scale_factor: 缩放因子

    返回:
    - 缩放后的线段列表
    """
    return [((x1 * scale_factor, y1 * scale_factor), (x2 * scale_factor, y2 * scale_factor))
            for ((x1, y1), (x2, y2)) in lines]

def scale_coordinates(coord, factor):
    """
    缩放单个坐标。

    参数:
    - coord: (x, y) 坐标
    - factor: 缩放因子

    返回:
    - 缩放后的坐标
    """
    x, y = coord
    return (x * factor, y * factor)

# ------------------------- GDS版图生成函数 -------------------------
def generate_gds_layout(s, t, s_path, t_path, res_via, random_rectangles, scale_factor=100):
    """
    生成GDS版图。

    参数:
     - s: 起点
    - t: 终点
    - s_path: 起点层布线
    - t_path: 终点层布线
    - res_via: 通孔列表
    - random_rectangles: 障碍物字典
    - scale_factor: 缩放因子

    返回:
    - 无
    """
    via_all_dict = {"single_via": res_via}
    lib = gdstk.Library()
    cell = lib.new_cell("Layout")

    # 缩放路径坐标
    h_path_all = scale_line_coordinates(s_path, scale_factor)
    v_path_all = scale_line_coordinates(t_path, scale_factor)

    # 缩放通孔坐标
    scaled_dict = {}
    for key, points in via_all_dict.items():
        coord, direction = points[0]
        scaled_coord = scale_coordinates(coord, scale_factor)
        scaled_dict[key] = [(scaled_coord, direction)]

    for i in h_path_all:
        start, global_end = i
        x_start, y_start = start
        x_global, y_global = global_end
        if y_start == y_global:
            # 对 X 坐标小的点进行向左偏移
            if x_start < x_global:
                x_start -= scale_factor
                x_global += scale_factor
            else:
                # 对 X 坐标小的点进行向左偏移
                x_global -= scale_factor
                x_start += scale_factor
            path = gdstk.FlexPath([(x_start, y_start), (x_global, y_global)], width=LINE_WIDTH_S * scale_factor, layer=63)
            cell.add(path)
        elif x_start == x_global:
            # 对 Y 坐标小的点进行向下偏移
            if y_start < y_global:
                y_start -= scale_factor
                y_global += scale_factor
            else:
                # 对 Y 坐标小的点进行向下偏移
                y_global -= scale_factor
                y_start += scale_factor
            path = gdstk.FlexPath([(x_start, y_start), (x_global, y_global)], width=LINE_WIDTH_S * scale_factor, layer=63)
            cell.add(path)

    for i in v_path_all:
        start, global_end = i
        x_start, y_start = start
        x_global, y_global = global_end
        if x_start == x_global:
            # 对 Y 坐标小的点进行向下偏移
            if y_start < y_global:
                y_start -= scale_factor
                y_global += scale_factor
            else:
                # 对 Y 坐标小的点进行向下偏移
                y_global -= scale_factor
                y_start += scale_factor
            path = gdstk.FlexPath([(x_start, y_start), (x_global, y_global)], width=LINE_WIDTH_T * scale_factor, layer=65)
            cell.add(path)
        elif y_start == y_global:
            # 对 X 坐标小的点进行向左偏移
            if x_start < x_global:
                x_start -= scale_factor
                x_global += scale_factor
            else:
                # 对 X 坐标小的点进行向左偏移
                x_global -= scale_factor
                x_start += scale_factor
            path = gdstk.FlexPath([(x_start, y_start), (x_global, y_global)], width=LINE_WIDTH_T * scale_factor, layer=65)
            cell.add(path)

    # 添加障碍物到GDS
    layer_mapping = {"1_layer": 63, "2_layer": 64, "3_layer": 65}
    for key, rects in random_rectangles.items():
        layer = layer_mapping.get(key, 63)
        for (x1, y1), (x2, y2) in rects:
            rect = gdstk.rectangle(
                (x1 * scale_factor, y1 * scale_factor),
                (x2 * scale_factor, y2 * scale_factor),
                layer=layer
            )
            cell.add(rect)

    # 保存GDS文件
    outfile = "./routing.gds"
    lib.write_gds(outfile)
    print(f"Layout saved to {outfile}.")

    # 合并已有的GDS与生成的GDS
    m1_m2 = pg.import_gds(filename='m2_m4.gds', cellname=None, flatten=False)
    post = pg.import_gds(filename='routing.gds', cellname=None, flatten=False)
    for polygon in post.polygons:
        polygon.scale(0.001, 0.001)

    # 插入通孔
    def insert_via(data):
        ref = post.add_ref(m1_m2)
        (coord, state) = data
        x, y = coord
        # 计算偏移量
        offset_x = x / 1000 - ref.x
        offset_y = y / 1000 - ref.y

        if state == 0:
            # 旋转90度并移动
            ref.rotate(90, center=(ref.x, ref.y))
        ref.move([offset_x, offset_y])

    for data in scaled_dict.get("single_via", []):
        insert_via(data)

    # 保存最终的GDS文件
    post.write_gds(
        "pattern_routing.gds",
        unit=1e-6,
        precision=1e-9,
        auto_rename=True,
        max_cellname_length=28,
        cellname="toplevel"
    )
    print("Final pattern routing GDS saved to pattern_routing.gds.")