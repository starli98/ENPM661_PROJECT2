# Github Link : https://github.com/starli98/ENPM661_PROJECT2
import numpy as np
import cv2
import math
from collections import deque
from queue import PriorityQueue

##############################################
# 1) Geometry & Obstacle Functions
##############################################

def map_to_bottom_left(x, y, width, height):
    # Convert model coordinates (origin at bottom-left) to image coordinates (origin at top-left)
    return (x, height - y)

def is_in_rectangle(x, y, x_min, x_max, y_min, y_max):
    # Return True if (x,y) lies within the given rectangle
    return (x_min <= x <= x_max) and (y_min <= y <= y_max)

def is_in_polygon(x, y, polygon):
    # Use ray-casting to determine if (x,y) is inside the polygon
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]             # current vertex
        x2, y2 = polygon[(i+1) % n]       # next vertex (wrap-around)
        if ((y1 > y) != (y2 > y)):        # if y is between y1 and y2
            x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x_intersect > x:         # if intersection is to the right
                inside = not inside     # toggle flag
    return inside

def is_in_circle(x, y, cx, cy, r):
    # Check if (x,y) is inside a circle with center (cx,cy) and radius r
    return (x - cx)**2 + (y - cy)**2 <= r*r

def is_in_ellipse_filled(x, y, cx, cy, rx, ry, angle_deg, start_deg, end_deg):
    # Translate point to ellipse-centered coordinates
    dx = x - cx
    dy = y - cy
    theta = -math.radians(angle_deg)  # rotation angle (negative)
    cosT = math.cos(theta)
    sinT = math.sin(theta)
    x_ell = dx * cosT - dy * sinT
    y_ell = dx * sinT + dy * cosT
    # Check ellipse equation
    if (x_ell**2) / (rx*rx) + (y_ell**2) / (ry*ry) > 1.0:
        return False
    # Calculate polar angle in ellipse frame
    ang = math.degrees(math.atan2(y_ell, x_ell))
    if ang < 0:
        ang += 360
    s = start_deg % 360
    e = end_deg % 360
    if s <= e:
        return s <= ang <= e
    else:
        return (ang >= s) or (ang <= e)

# Obstacle definitions based on letters/digits (No Clearance)
def is_in_E(x, y):
    r1 = is_in_rectangle(x, y, 50, 70, 60, 240)
    r2 = is_in_rectangle(x, y, 70, 130, 220, 240)
    r3 = is_in_rectangle(x, y, 70, 110, 140, 160)
    r4 = is_in_rectangle(x, y, 70, 130, 60, 80)
    return r1 or r2 or r3 or r4

def is_in_N(x, y):
    r1 = is_in_rectangle(x, y, 200, 220, 60, 240)
    r2 = is_in_rectangle(x, y, 280, 300, 60, 240)
    diag_poly = [(290,60), (270,60), (210,240), (230,240)]
    diag = is_in_polygon(x, y, diag_poly)
    return r1 or r2 or diag

def is_in_P(x, y):
    rect_p = is_in_rectangle(x, y, 350, 370, 60, 240)
    ell_p  = is_in_ellipse_filled(x, y, 370, 200, 40, 40, 0, -90, 90)
    return rect_p or ell_p

def is_in_M(x, y):
    r1 = is_in_rectangle(x, y, 460, 480, 60, 240)
    r2 = is_in_rectangle(x, y, 580, 600, 60, 240)
    left_diag = [(470,240), (490,240), (540,60), (520,60)]
    ld = is_in_polygon(x, y, left_diag)
    right_diag = [(540,60), (520,60), (570,240), (590,240)]
    rd = is_in_polygon(x, y, right_diag)
    return r1 or r2 or ld or rd

def is_in_6_first(x, y):
    bottom_circle = is_in_circle(x, y, 695, 105, 50)
    radius_top_arc = 150
    center_top_arc = (795, 105)
    rect_size = 10
    for deg in range(120, 181):
        rad = math.radians(deg)
        px = center_top_arc[0] + radius_top_arc * math.cos(rad)
        py = center_top_arc[1] + radius_top_arc * math.sin(rad)
        x_min = px - rect_size/2
        x_max = px + rect_size/2
        y_min = py - rect_size/2
        y_max = py + rect_size/2
        if (x_min <= x <= x_max) and (y_min <= y <= y_max):
            return True
    return bottom_circle

def is_in_6_second(x, y):
    bottom_circle = is_in_circle(x, y, 845, 105, 50)
    radius_top_arc = 150
    center_top_arc = (945, 105)
    rect_size = 10
    for deg in range(120, 181):
        rad = math.radians(deg)
        px = center_top_arc[0] + radius_top_arc * math.cos(rad)
        py = center_top_arc[1] + radius_top_arc * math.sin(rad)
        x_min = px - rect_size/2
        x_max = px + rect_size/2
        y_min = py - rect_size/2
        y_max = py + rect_size/2
        if (x_min <= x <= x_max) and (y_min <= y <= y_max):
            return True
    return bottom_circle

def is_in_1(x, y):
    return is_in_rectangle(x, y, 950, 970, 60, 240)

def is_obstacle(x, y):
    # Check if (x,y) falls within any of the obstacles
    return (is_in_E(x, y) or
            is_in_N(x, y) or
            is_in_P(x, y) or
            is_in_M(x, y) or
            is_in_6_first(x, y) or
            is_in_6_second(x, y) or
            is_in_1(x, y))

##############################################
# 2) BFS Search (8-connected, stop when goal reached)
##############################################
def is_free(obs_map_original, pt):
    # Check if pt is free (white) in the original map
    x, y = pt
    H, W = obs_map_original.shape[:2]
    if 0 <= x < W and 0 <= y < H:
        return (obs_map_original[y, x] == [255,255,255]).all()
    return False

def get_neighbors(obs_map_original, pt):
    # Get 8-connected neighbors that are free in the original map
    x, y = pt
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if is_free(obs_map_original, (nx, ny)):
                neighbors.append((nx, ny))
    return neighbors

def bfs_search(start, goal, obs_map_original, interval=200, video_writer=None):
    # Create a display copy for visualization
    obs_map_display = obs_map_original.copy()
    queue = deque([start])
    visited = set([start])
    came_from = {start: None}
    explored_count = 0

    cv2.namedWindow("BFS Visualization", cv2.WINDOW_NORMAL)

    while queue:
        cur = queue.popleft()
        if cur == goal:
            # Backtrack the path from goal to start
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path, obs_map_display

        # Visualize current node in green
        obs_map_display[cur[1], cur[0]] = (0,255,0)
        explored_count += 1

        # Mark start (blue) and goal (red)
        cv2.circle(obs_map_display, start, 5, (255,0,0), -1)
        cv2.circle(obs_map_display, goal, 5, (0,0,255), -1)

        if explored_count % interval == 0:
            cv2.imshow("BFS Visualization", obs_map_display)
            cv2.waitKey(1)
            if video_writer is not None:
                video_writer.write(obs_map_display)

        # Expand all 8-connected neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nxt = (cur[0] + dx, cur[1] + dy)
                if nxt not in visited and is_free(obs_map_original, nxt):
                    visited.add(nxt)
                    came_from[nxt] = cur
                    queue.append(nxt)
    return None, obs_map_display

##############################################
# 3) Terminal Input for Start/Goal Selection
##############################################
def input_point(prompt, default, obs_map_original, width, height):
    # Prompt user to enter a point in "x,y" format. If empty, use default.
    while True:
        user_input = input(f"{prompt} (default: {default[0]},{default[1]}): ")
        if user_input.strip() == "":
            point = default
        else:
            try:
                x_str, y_str = user_input.split(',')
                point = (int(x_str.strip()), int(y_str.strip()))
            except:
                print("Invalid format. Please enter as x,y")
                continue
        # Convert model coordinate to image coordinate.
        pt_img = map_to_bottom_left(point[0], point[1], width, height)
        # Check if the point is free (white) in the original map.
        if not (obs_map_original[pt_img[1], pt_img[0]] == [255,255,255]).all():
            print("CANNOT CHOOSE")
        else:
            return pt_img

##############################################
# 4) Main: Terminal input for start/goal, run BFS, visualize path (3x3 block), and save MP4
##############################################
def main():
    global width, height
    width, height = 1000, 300

    # Build the original map: white background with black obstacles.
    obs_map_original = np.ones((height, width, 3), dtype=np.uint8) * 255
    for py in range(height):
        for px in range(width):
            # Convert image coordinate to model coordinate
            y_model = height - 1 - py
            if is_obstacle(px, y_model):
                obs_map_original[py, px] = (0,0,0)

    # Terminal input for start & goal (model coordinates)
    # Recommended defaults: start = (33,243), goal = (984,199)
    start_img = input_point("Enter start point (x,y)", (29,156), obs_map_original, width, height)
    goal_img  = input_point("Enter goal point (x,y)", (922,149), obs_map_original, width, height)

    print("Start (image coordinate):", start_img, "Color =", obs_map_original[start_img[1], start_img[0]])
    print("Goal (image coordinate):", goal_img, "Color =", obs_map_original[goal_img[1], goal_img[0]])

    # Create VideoWriter to save the visualization as MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_out = cv2.VideoWriter('optimal_path.mp4', fourcc, 30.0, (width, height))

    # Run BFS search
    path, obs_map_display = bfs_search(start_img, goal_img, obs_map_original, interval=200, video_writer=video_out)
    if path is None:
        print("No path found!")
        cv2.imshow("BFS Visualization", obs_map_display)
        cv2.waitKey(0)
        video_out.write(obs_map_display)
        video_out.release()
        cv2.destroyAllWindows()
        return

    # Print the path waypoints to terminal
    print("Path found, length =", len(path))
    print("Waypoints (x, y):")
    for node in path:
        print(node)

    # Draw the path on the display map with 3x3 blocks (red)
    block_size = 3
    half_bs = block_size // 2  # half_bs = 1 for 3x3 block
    path_color = (0,0,255)      # Red color

    for node in path:
        cx, cy = node
        # Draw a 3x3 block centered at the node
        for dx in range(-half_bs, half_bs+1):
            for dy in range(-half_bs, half_bs+1):
                nx, ny = cx+dx, cy+dy
                if 0 <= nx < width and 0 <= ny < height:
                    obs_map_display[ny, nx] = path_color
        cv2.imshow("BFS Visualization", obs_map_display)
        cv2.waitKey(1)
        video_out.write(obs_map_display)

    # Final display and save final frame to video
    cv2.imshow("BFS Visualization", obs_map_display)
    cv2.waitKey(0)
    video_out.write(obs_map_display)
    video_out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
