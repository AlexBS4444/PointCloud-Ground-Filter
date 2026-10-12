#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSF (Cloth Simulation Filter) 算法逻辑验证脚本
与 C++ 实现 CSFFilter 保持一致的算法步骤，用于在无法编译 C++ 时验证功能逻辑。
"""

import math
import os


def read_xyz(path):
    pts = []
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 3:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return pts


def csf_filter(points, cloth_resolution=1.0, max_iterations=500,
               classification_threshold=0.5, convergence_threshold=0.005,
               rigidness=3, time_step=0.65):
    """
    布料模拟滤波 Python 参考实现
    返回：ground_mask（与 points 等长的 bool 列表）
    """
    if not points:
        return []

    # 1. 预处理：剔除无效点并翻转 Z
    valid = []
    for i, p in enumerate(points):
        if all(math.isfinite(v) for v in p):
            valid.append((i, [p[0], p[1], -p[2]]))

    if not valid:
        return [False] * len(points)

    flipped = [v[1] for v in valid]

    # 计算包围盒
    xs = [p[0] for p in flipped]
    ys = [p[1] for p in flipped]
    zs = [p[2] for p in flipped]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_max = max(zs)

    # 2. 建立点云碰撞格网（记录每个格网最大翻转高程）
    cols = int(math.ceil((x_max - x_min) / cloth_resolution)) + 1
    rows = int(math.ceil((y_max - y_min) / cloth_resolution)) + 1
    cols = max(1, cols)
    rows = max(1, rows)

    collision = [[-math.inf for _ in range(cols)] for _ in range(rows)]
    for p in flipped:
        c = int((p[0] - x_min) / cloth_resolution)
        r = int((p[1] - y_min) / cloth_resolution)
        c = max(0, min(c, cols - 1))
        r = max(0, min(r, rows - 1))
        collision[r][c] = max(collision[r][c], p[2])

    # 初始化布料格网
    nodes = []
    start_z = z_max + 2.0 * cloth_resolution
    for r in range(rows):
        for c in range(cols):
            nodes.append({
                'x': x_min + c * cloth_resolution,
                'y': y_min + r * cloth_resolution,
                'z': start_z,
                'movable': True
            })

    def get_collision(r, c):
        if 0 <= r < rows and 0 <= c < cols:
            return collision[r][c]
        return -math.inf

    # 3-5. 迭代模拟
    rigid_factor = {1: 0.1, 2: 0.3, 3: 0.5}.get(rigidness, 0.5)
    gravity = 9.8

    for _ in range(max_iterations):
        max_change = 0.0

        # 外力 + 碰撞
        for node in nodes:
            if not node['movable']:
                continue
            z_old = node['z']
            z_new = z_old - gravity * time_step * time_step
            c = int((node['x'] - x_min) / cloth_resolution)
            r = int((node['y'] - y_min) / cloth_resolution)
            cz = get_collision(r, c)
            if z_new <= cz:
                node['z'] = cz
                node['movable'] = False
            else:
                node['z'] = z_new
            max_change = max(max_change, abs(node['z'] - z_old))

        # 内力（4-邻域约束）
        new_z = []
        for idx, node in enumerate(nodes):
            if not node['movable']:
                new_z.append(node['z'])
                continue
            r = idx // cols
            c = idx % cols
            total = 0.0
            cnt = 0
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    total += nodes[nr * cols + nc]['z']
                    cnt += 1
            if cnt > 0:
                avg = total / cnt
                new_z.append(node['z'] + (avg - node['z']) * rigid_factor)
            else:
                new_z.append(node['z'])

        for i, node in enumerate(nodes):
            if not node['movable']:
                continue
            change = abs(new_z[i] - node['z'])
            max_change = max(max_change, change)
            node['z'] = new_z[i]

        if max_change < convergence_threshold:
            break

    # 6. 分类
    ground_mask = [False] * len(points)
    for orig_idx, p in valid:
        c = int((p[0] - x_min) / cloth_resolution)
        r = int((p[1] - y_min) / cloth_resolution)
        c = max(0, min(c, cols - 1))
        r = max(0, min(r, rows - 1))
        cloth_h = nodes[r * cols + c]['z']
        diff = abs(p[2] - cloth_h)
        if diff < classification_threshold:
            ground_mask[orig_idx] = True

    return ground_mask


def test_flat_plane():
    """测试 1：完全平坦地面，所有点应为地面点"""
    pts = read_xyz('test_cloud.txt')
    mask = csf_filter(pts, cloth_resolution=1.0, max_iterations=500,
                      classification_threshold=0.5)
    ground = sum(mask)
    print(f"[测试1] 平坦地面：{ground}/{len(pts)} 被识别为地面点")
    assert ground == len(pts), "平坦地面应全部识别为地面点"


def test_slope_with_objects():
    """测试 2：倾斜地面 + 上方地物，验证非地面点被正确剔除"""
    pts = []
    # 倾斜地面：z = 0.1 * x
    for x in [i * 0.5 for i in range(20)]:
        for y in [j * 0.5 for j in range(20)]:
            pts.append([x, y, 0.1 * x])

    # 在 (5,5) 附近放置一个高 2m 的“建筑物”
    for x in [4.5, 5.0, 5.5]:
        for y in [4.5, 5.0, 5.5]:
            for h in [0.5, 1.0, 1.5, 2.0]:
                pts.append([x, y, 0.1 * x + h])

    mask = csf_filter(pts, cloth_resolution=0.5, max_iterations=500,
                      classification_threshold=0.3)
    ground = sum(mask)
    non_ground = len(pts) - ground
    print(f"[测试2] 倾斜地面+建筑物：地面点 {ground}，非地面点 {non_ground}")
    assert non_ground >= 10, "应识别出一定数量的非地面点"


def test_empty_input():
    """测试 3：空输入不崩溃"""
    mask = csf_filter([])
    print(f"[测试3] 空输入：返回 mask 长度 {len(mask)}")
    assert len(mask) == 0


if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    test_flat_plane()
    test_slope_with_objects()
    test_empty_input()
    print("\n所有 CSF 算法逻辑测试通过。")
