Description: This Python program demonstrates a Breadth-First Search (BFS) algorithm for path planning in a 2D grid. The grid is created with a white background and obstacles drawn in black, which are defined using various geometric shapes (such as rectangles, circles, polygons, and ellipses). The code visualizes the exploration process in a window and saves an MP4 video ("optimal_path.mp4") of the search and final path.

Dependencies & Libraries: • Python 3.x • numpy – for numerical array operations. • OpenCV (cv2) – for image processing, visualization, and video creation. • math – for mathematical computations (included in the Python Standard Library). • collections – for deque (included in the Python Standard Library). • queue – for PriorityQueue (included in the Python Standard Library).

Installation:

Ensure that Python 3.x is installed on your system.
Install the required external libraries using pip: pip install numpy opencv-python
Usage Instructions:

Open a terminal (or command prompt) and navigate to the directory containing the BFS_Xinze_Li.py file.
Run the program by executing: python BFS_Xinze_Li.py
The program will build a 1000x300 map with obstacles. A window named "BFS Visualization" will appear.
The terminal will prompt you to input the start and goal points in the format “x,y”. These coordinates are provided in model space (origin at the bottom-left). The program converts these coordinates to image space (origin at the top-left).
If you press Enter without typing any values, default points will be used: • Default start point: (29,156) • Default goal point: (922,149)
Once the start and goal points are provided and validated (ensuring they are in free space), the BFS algorithm begins to explore the map.
The exploration is visualized in real time, with the current node marked in green, the start point in blue, and the goal point in red.
When a path is found, the waypoints are printed to the terminal, and the final path is drawn in red (using 3x3 blocks) on the map.
A video of the entire process is saved as “optimal_path.mp4” in the same directory.
Notes: • Ensure that the chosen start and goal points are within free (white) space. If not, the program will prompt you to select different coordinates. • The obstacles are defined in the code using geometric functions. For modifications, review the obstacle functions (e.g., is_in_E, is_in_N, etc.) in the code. • The program uses OpenCV to create a window for visualization and to write the video file, so the process may vary slightly depending on your system’s graphical capabilities.

