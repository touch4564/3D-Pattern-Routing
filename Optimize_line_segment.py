"""

路径线段优化，去除冗杂线段

"""
from typing import List, Tuple

# 定义类型别名
Segment = Tuple[Tuple[float, float], Tuple[float, float]]
Rectangle = Tuple[Tuple[float, float], Tuple[float, float]]

# ------------------------- 优化穿过引脚区域的函数 -------------------------
def rectangle_from_diagonal(p1, p2):
    """
    根据矩形的对角线两个点，返回标准格式的矩形坐标 (x_min, y_min, x_max, y_max)。

    参数:
        p1 (tuple): 第一个对角线点 (x1, y1)。
        p2 (tuple): 第二个对角线点 (x2, y2)。

    返回:
        tuple: 标准格式的矩形坐标 (x_min, y_min, x_max, y_max)。
    """
    x_min, x_max = sorted([p1[0], p2[0]])
    y_min, y_max = sorted([p1[1], p2[1]])
    return x_min, y_min, x_max, y_max


def liang_barsky(line_seg, rect):
    """
    使用 Liang-Barsky 算法裁剪线段与矩形的交集部分，并返回裁剪后的线段列表。

    参数:
        line_seg (tuple): 线段的两个端点 ((x0, y0), (x1, y1))。
        rect (tuple): 矩形的标准格式坐标 (x_min, y_min, x_max, y_max)。

    返回:
        list: 裁剪后的线段列表，每个线段的坐标为整数。
    """
    x0, y0 = line_seg[0]
    x1, y1 = line_seg[1]

    x_min, y_min, x_max, y_max = rect

    dx = x1 - x0
    dy = y1 - y0

    # 定义参数 p 和 q，用于计算参数 t
    p = [-dx, dx, -dy, dy]
    q = [x0 - x_min, x_max - x0, y0 - y_min, y_max - y0]

    u1, u2 = 0.0, 1.0  # 参数 t 的初始范围

    for edge in range(4):
        if p[edge] == 0:
            if q[edge] < 0:
                # 线段与矩形边缘平行且在矩形外部，无交集
                return [line_seg]
            else:
                # 线段与矩形边缘平行且在矩形内部，继续检查其他边
                continue
        t = q[edge] / p[edge]
        if p[edge] < 0:
            if t > u1:
                u1 = t  # 更新下界
        else:
            if t < u2:
                u2 = t  # 更新上界
        if u1 > u2:
            # 参数范围无效，无交集
            return [line_seg]

    # 存在交集，需要裁剪
    segments = []
    if u1 > 0:
        # 裁剪前部分
        segments.append((
            (x0, y0),
            (x0 + u1 * dx, y0 + u1 * dy)
        ))
    if u2 < 1:
        # 裁剪后部分
        segments.append((
            (x0 + u2 * dx, y0 + u2 * dy),
            (x1, y1)
        ))

    # 将裁剪后的线段坐标转换为整数
    return [(
        (int(round(seg[0][0])), int(round(seg[0][1]))),
        (int(round(seg[1][0])), int(round(seg[1][1])))
    ) for seg in segments]


def process_line_segments(line_segments, rectangles):
    """
    处理多个线段，裁剪它们与多个矩形的交集部分，并返回处理后的线段列表。

    参数:
        line_segments (list): 线段列表，每个线段为 ((x0, y0), (x1, y1))。
        rectangles (list): 矩形列表，每个矩形为 ((p1, p2, layer))。

    返回:
        list: 处理后的线段列表，所有坐标均为整数。
    """
    # 将矩形转换为标准格式
    standard_rects = []
    for rect in rectangles:
        p1, p2, layer = rect
        standard_rects.append(rectangle_from_diagonal(p1, p2))

    result_segments = []
    for line_seg in line_segments:
        # 初始化需要处理的线段列表
        segments_to_process = [line_seg]
        for rect in standard_rects:
            new_segments = []
            for seg in segments_to_process:
                # 对每个线段应用 Liang-Barsky 裁剪算法
                clipped_segments = liang_barsky(seg, rect)
                new_segments.extend(clipped_segments)
            segments_to_process = new_segments
        result_segments.extend(segments_to_process)

    return result_segments

# ------------------------- 优化与引脚区域重叠的线段的函数 -------------------------
def is_vertical(segment: Segment) -> bool:
    """判断线段是否为垂直线段"""
    (x1, y1), (x2, y2) = segment
    return x1 == x2


def is_horizontal(segment: Segment) -> bool:
    """判断线段是否为水平线段"""
    (x1, y1), (x2, y2) = segment
    return y1 == y2


def get_overlap_interval(seg1: Segment, seg2: Segment, orientation: str) -> Tuple[float, float]:
    """
    计算两个线段在对应轴上的重叠区间。
    orientation: 'vertical' 或 'horizontal'
    """
    if orientation == 'vertical':
        # 对于垂直线段，比较 y 轴
        y1_min, y1_max = sorted([seg1[0][1], seg1[1][1]])
        y2_min, y2_max = sorted([seg2[0][1], seg2[1][1]])
        overlap_min = max(y1_min, y2_min)
        overlap_max = min(y1_max, y2_max)
    else:
        # 对于水平线段，比较 x 轴
        x1_min, x1_max = sorted([seg1[0][0], seg1[1][0]])
        x2_min, x2_max = sorted([seg2[0][0], seg2[1][0]])
        overlap_min = max(x1_min, x2_min)
        overlap_max = min(x1_max, x2_max)

    if overlap_min <= overlap_max:
        return (overlap_min, overlap_max)
    else:
        return None  # 无重叠


def subtract_interval(seg: Segment, overlap: Tuple[float, float], orientation: str) -> List[Segment]:
    """
    从线段中去除重叠区间，返回剩余部分的线段列表。
    """
    remaining_segments = []
    if orientation == 'vertical':
        x = seg[0][0]
        y1, y2 = sorted([seg[0][1], seg[1][1]])
        overlap_start, overlap_end = overlap
        # 上半部分
        if y1 < overlap_start:
            remaining_segments.append(((x, y1), (x, overlap_start)))
        # 下半部分
        if overlap_end < y2:
            remaining_segments.append(((x, overlap_end), (x, y2)))
    else:
        y = seg[0][1]
        x1, x2 = sorted([seg[0][0], seg[1][0]])
        overlap_start, overlap_end = overlap
        # 左半部分
        if x1 < overlap_start:
            remaining_segments.append(((x1, y), (overlap_start, y)))
        # 右半部分
        if overlap_end < x2:
            remaining_segments.append(((overlap_end, y), (x2, y)))

    return remaining_segments


def remove_overlaps(segments: List[Segment], rect: Rectangle) -> List[Segment]:
    """
    检测出线段与矩形重叠部分，并将这部分从线段列表中去除。
    """
    (rx1, ry1), (rx2, ry2), z = rect
    # 确保矩形的坐标是（左下，右上）
    rx_min, rx_max = sorted([rx1, rx2])
    ry_min, ry_max = sorted([ry1, ry2])

    # 定义矩形的四条边
    rect_edges = [
        ((rx_min, ry_min), (rx_min, ry_max)),  # 左边
        ((rx_max, ry_min), (rx_max, ry_max)),  # 右边
        ((rx_min, ry_min), (rx_max, ry_min)),  # 下边
        ((rx_min, ry_max), (rx_max, ry_max)),  # 上边
    ]

    result_segments = []

    for seg in segments:
        temp_segments = [seg]  # 当前线段可能被分割成多个部分
        for edge in rect_edges:
            new_temp_segments = []
            for current_seg in temp_segments:
                # 判断当前线段与矩形边是否平行且可能重叠
                if is_vertical(current_seg) and is_vertical(edge):
                    if current_seg[0][0] != edge[0][0]:
                        # 平行但不同x坐标，不重叠
                        new_temp_segments.append(current_seg)
                        continue
                    # 计算重叠区间
                    overlap = get_overlap_interval(current_seg, edge, 'vertical')
                elif is_horizontal(current_seg) and is_horizontal(edge):
                    if current_seg[0][1] != edge[0][1]:
                        # 平行但不同y坐标，不重叠
                        new_temp_segments.append(current_seg)
                        continue
                    # 计算重叠区间
                    overlap = get_overlap_interval(current_seg, edge, 'horizontal')
                else:
                    # 不平行，忽略（根据初始需求，仅处理平行线段）
                    overlap = None

                if overlap:
                    # 从当前线段中去除重叠部分
                    remaining = subtract_interval(current_seg, overlap,
                                                  'vertical' if is_vertical(current_seg) else 'horizontal')
                    new_temp_segments.extend(remaining)
                else:
                    # 无重叠，保留当前线段
                    new_temp_segments.append(current_seg)
            temp_segments = new_temp_segments  # 更新待处理的线段

        result_segments.extend(temp_segments)  # 添加处理后的线段

    return result_segments


def remove_overlaps_from_paths(s_path, t_path, s, t):
    """
    从起点路径和终点路径中移除与起点和终点的重叠部分。

    参数:
        s_path (list): 起点路径列表，包含路径上的各个节点。
        t_path (list): 终点路径列表，包含路径上的各个节点。
        s (tuple): 起点信息，格式为 (x, y, z)，其中 z 可能表示层级或其他属性。
        t (tuple): 终点信息，格式为 (x, y, z)，其中 z 可能表示层级或其他属性。

    返回:
        tuple: 更新后的起点路径和终点路径，格式为 (s_path, t_path)。
    """
    # 检查起点和终点的第三个元素（可能表示层级）是否不同
    if s[2] != t[2]:
        # 如果起点和终点的层级不同，只需分别从各自的路径中移除与自身的重叠部分
        s_path = remove_overlaps(s_path, s)  # 移除 s_path 中与起点 s 重叠的部分
        t_path = remove_overlaps(t_path, t)  # 移除 s_path 中与终点 t 重叠的部分

        # # # 移除 s_path 中穿过起点的线段
        # s_path = process_line_segments(s_path, rectangles = [s, t])
        # #
        # t_path = process_line_segments(t_path, rectangles = [s, t])


    else:
        # 如果起点和终点的层级相同，需考虑两者之间可能的重叠
        # 先从起点路径中移除与起点 s 重叠的部分
        s_path = remove_overlaps(s_path, s)

        # 移除 s_path 中穿过起点的线段
        # s_path = process_line_segments(s_path, rectangles=[s, t])

        # 再从起点路径中移除与终点 t 重叠的部分
        s_path = remove_overlaps(s_path, t)

    # 返回处理后的起点路径和终点路径
    return s_path, t_path


# 示例用法
if __name__ == "__main__":
    segments = [((683, 500), (683, 3000)), ((683, 3000), (800, 3000))]
    rect = ((500, 3000), (4000, 4000), 2)

    print("原始线段列表:")
    for seg in segments:
        print(seg)

    updated_segments = remove_overlaps(segments, rect)

    print("\n去除与矩形重叠部分后的线段列表:")
    for seg in updated_segments:
        print(seg)


    # 输入数据：线段列表
    line_segments = [
        ((1600, 1000), (1600, 2430)),
        ((3000, 2430), (3000, 3800))
    ]

    # 输入数据：矩形列表，每个矩形为 (p1, p2, angle)
    s = ((1000, 1000), (2000, 1200), 0)
    t = ((3000, 3800), (4000, 4000), 0)
    rectangles = [s, t]

    # 处理线段
    processed_segments = process_line_segments(line_segments, rectangles)

    # 输出结果
    print(processed_segments)
