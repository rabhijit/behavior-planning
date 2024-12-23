import math
import open3d as o3d
import pandas as pd
import numpy as np
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from geopy.distance import geodesic
import xml.etree.ElementTree as ET

LANE_WIDTH = 1.524 # 1.524m = 5ft. Lane width from left end to right end is LANE_WIDTH * 2 = 10ft.
AREA = 8

# Load csv and point cloud
points_csv = pd.read_csv("area-{}/vector_map/point.csv".format(AREA))
lanes_csv = pd.read_csv("area-{}/vector_map/lane.csv".format(AREA))
nodes_csv = pd.read_csv("area-{}/vector_map/node.csv".format(AREA))
pcd_3d = o3d.io.read_point_cloud("area-{}/area-ds.pcd".format(AREA))
pcd_3d_points = np.asarray(pcd_3d.points)
pcd_3d_points[:, 2] = 0 # set z = 0

lanelet2_file = "area-{}/lanelet2-area-{}.osm".format(AREA, AREA)

# Extract 3D coordinates (Bx, Ly, H)
pids = points_csv["PID"].values
points = points_csv[["Bx", "Ly", "H", "B", "L"]].values
points[:, 2] = 0 # set H as 0
lanes = lanes_csv[["LnID", "FLID"]].values

# Create an Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points[:, :3])
colors = np.zeros((points.shape[0], 3))  # Black (R=0, G=0, B=0)

curr_lnid = 1
all_lane_segments = [] # list of lists - all the LnIDs in a single lane segment

def get_point(nid):
    index = np.where(nodes_csv["NID"] == nid)[0]
    if index.size > 0:
        return nodes_csv["PID"][index[0]]  # (Bx, Ly)
    else:
        raise ValueError(f"NID {nid} not found in node data")

def get_coordinates(pid):
    index = np.where(pids == pid)[0]
    if index.size > 0:
        return (points[index[0], 0], points[index[0], 1], points[index[0], 2])  # (Bx, Ly, H)
    else:
        raise ValueError(f"PID {pid} not found in point data")
    
def get_lat_long(pid):
    index = np.where(pids == pid)[0]
    if index.size > 0:
        return (points[index[0], 3], points[index[0], 4])  # (B, L)
    else:
        raise ValueError(f"PID {pid} not found in point data")
    
def calculate_boundaries(Bx1, Ly1, H1, Bx2, Ly2, H2, offset=LANE_WIDTH):
    # Calculate the direction vector
    direction_vector = np.array([Bx2 - Bx1, Ly2 - Ly1])
    
    # Normalize the direction vector
    norm = np.linalg.norm(direction_vector)
    if norm == 0:
        return (Bx1, Ly1), (Bx1, Ly1), (Bx2, Ly2), (Bx2, Ly2)
    direction_vector = direction_vector / norm
    
    # Calculate the perpendicular vector
    perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])
    
    # Scale the perpendicular vector to 5 feet
    offset_vector = perpendicular_vector * offset
    
    # Calculate the boundary points
    boundary1_start = np.array([Bx1, Ly1]) + offset_vector
    boundary1_end = np.array([Bx2, Ly2]) + offset_vector
    boundary2_start = np.array([Bx1, Ly1]) - offset_vector
    boundary2_end = np.array([Bx2, Ly2]) - offset_vector

    boundary1_start = np.append(boundary1_start, H1)
    boundary1_end = np.append(boundary1_end, H1)
    boundary2_start = np.append(boundary2_start, H2)
    boundary2_end = np.append(boundary2_end, H2)
    
    return boundary1_start, boundary1_end, boundary2_start, boundary2_end

def point_in_polygon(point, polygon_points):
    x, y = point
    n = len(polygon_points)
    inside = False

    p1x, p1y = polygon_points[0]
    for i in range(n + 1):
        p2x, p2y = polygon_points[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def calculate_boundary_coordinates(Bx1, Ly1, Bx2, Ly2):
    """
    Calculate left and right boundary points in Cartesian coordinates (Bx, Ly).
    """
    # Step 1: Calculate the direction vector
    direction_vector = np.array([Bx2 - Bx1, Ly2 - Ly1])
    
    # Step 2: Normalize the direction vector
    norm = np.linalg.norm(direction_vector)
    if norm == 0:
        return None, None  # Ignore if two points are the same
    direction_vector /= norm
    
    # Step 3: Calculate the perpendicular vector
    perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])
    
    # Step 4: Scale perpendicular vector for the lane width
    offset_vector = perpendicular_vector * LANE_WIDTH  # Offset is half-lane width (1.524m)
    
    # Step 5: Calculate boundary points
    left_boundary_start = [Bx1 + offset_vector[0], Ly1 + offset_vector[1]]
    right_boundary_start = [Bx1 - offset_vector[0], Ly1 - offset_vector[1]]
    
    return left_boundary_start, right_boundary_start

reflection_matrix_y = np.array([[-1,  0,  0],
                                [ 0,  1,  0],
                                [ 0,  0,  1]])

rotation_matrix_90 = np.array([[0, 1, 0],
                               [-1, 0, 0],
                               [0, 0, 1]])

combined_matrix = rotation_matrix_90 @ reflection_matrix_y
pcd_3d_points_rotated = pcd_3d_points @ combined_matrix.T

# Update the point cloud with the rotated points
pcd_3d.points = o3d.utility.Vector3dVector(pcd_3d_points_rotated)

midpoints_pcd = o3d.geometry.PointCloud()
midpoints_pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # Add midpoints directly
midpoint_colors = np.tile([1, 0, 0], (points.shape[0], 1))  # Red for midpoints
midpoints_pcd.colors = o3d.utility.Vector3dVector(midpoint_colors)

for lnid in lanes_csv["LnID"]:
    if not any(lnid in segment for segment in all_lane_segments):
        curr_lnid = lnid
        lane_segment = []
        while curr_lnid != 0:
            lane_segment.append(curr_lnid)
            index = lanes_csv.index[lanes_csv["LnID"] == curr_lnid].tolist()
            if index:  # If the LnID is found
                curr_lnid = int(lanes_csv["FLID"][index[0]])
            else:
                curr_lnid = 0
        all_lane_segments.append(lane_segment)

all_lane_segment_points = [] # list of lists - all the PIDs that make up each lane segment

for lane_segment in all_lane_segments:
    lane_segment_points = []
    for lnid in lane_segment:
        index = lanes_csv.index[lanes_csv["LnID"] == lnid].tolist()
        if index:  # If the LnID is found
            bnid = int(lanes_csv["BNID"][index[0]])
            lane_segment_points.append(bnid)
    lane_segment_points.append(int(lanes_csv["FNID"][index[0]]))
    all_lane_segment_points.append(lane_segment_points)

# Step 2: Recompute boundaries with updated height values
boundary_points_flat = []  # Flattened list of boundary points
boundary_lines = []        # Line indices for the LineSet
lane_segment_boundaries = []  # To store boundaries for each lane segment
combined_boundary_polygon = Polygon()

for lane_segment_points in all_lane_segment_points:
    prev_left_end = None  # To store the previous segment's left boundary end
    prev_right_end = None  # To store the previous segment's right boundary end
    segment_boundaries = {
        "centerline": [],
        "left_boundary": [],
        "right_boundary": []
    }

    for i in range(len(lane_segment_points) - 1):
        bnid1 = lane_segment_points[i]
        bnid2 = lane_segment_points[i + 1]
        
        try:
            # Get the points
            bpid1 = get_point(bnid1)
            bpid2 = get_point(bnid2)
            # Get the midpoints (Bx, Ly, H)
            Bx1, Ly1, H1 = get_coordinates(bpid1)
            Bx2, Ly2, H2 = get_coordinates(bpid2)
            B1, L1 = get_lat_long(bpid1)
            B2, L2 = get_lat_long(bpid2)
        except ValueError as e:
            print(e)
            continue

        # Compute boundaries
        boundary1_start, boundary1_end, boundary2_start, boundary2_end = calculate_boundaries(
            Bx1, Ly1, H1, Bx2, Ly2, H2
        )

        # current_polygon = Polygon([
        #     boundary1_start[:2],  # Left boundary start
        #     boundary1_end[:2],    # Left boundary end
        #     boundary2_end[:2],    # Right boundary end
        #     boundary2_start[:2]   # Right boundary start
        # ])

        line1 = LineString([boundary1_start[:2], boundary1_end[:2]])  # Left boundary
        line2 = LineString([boundary2_start[:2], boundary2_end[:2]])  # Right boundary

        skip_line1 = False
        # if combined_boundary_polygon.is_valid and combined_boundary_polygon.intersects(line1):
        #     intersection = combined_boundary_polygon.intersection(line1)
        #     overlap_ratio = intersection.length / line1.length
        #     if overlap_ratio > 0.15:
        #         #print(f"Left boundary line overlaps {overlap_ratio * 100:.2f}% - Skipping line.")
        #         skip_line1 = True

        # Check overlap for line2
        skip_line2 = False
        # if combined_boundary_polygon.is_valid and combined_boundary_polygon.intersects(line2):
        #     intersection = combined_boundary_polygon.intersection(line2)
        #     overlap_ratio = intersection.length / line2.length
        #     if overlap_ratio > 0.15:
        #         #print(f"Right boundary line overlaps {overlap_ratio * 100:.2f}% - Skipping line.")
        #         skip_line2 = True

        # Merge the current polygon with the combined boundary polygon
        # combined_boundary_polygon = unary_union([combined_boundary_polygon, current_polygon])

        # Ensure continuity with previous segment's end points
        if prev_left_end is not None and prev_right_end is not None:
           boundary1_start = prev_left_end
           boundary2_start = prev_right_end

        if not skip_line1:
            boundary_points_flat.append(boundary1_start)
            boundary_points_flat.append(boundary1_end)
            n = len(boundary_points_flat)
            boundary_lines.append([n - 2, n - 1])

        if not skip_line2:
            boundary_points_flat.append(boundary2_start)
            boundary_points_flat.append(boundary2_end)
            n = len(boundary_points_flat)
            boundary_lines.append([n - 2, n - 1])

        # Update previous segment's end points
        prev_left_end = boundary1_end if not skip_line1 else None
        prev_right_end = boundary2_end if not skip_line2 else None

        left_boundary, right_boundary = calculate_boundary_coordinates(B1, L1, B2, L2)

        segment_boundaries["centerline"].append([Bx1, Ly1, H1])
        #if left_boundary and right_boundary:
        segment_boundaries["left_boundary"].append([boundary1_start[0], boundary1_start[1], H1])
        segment_boundaries["right_boundary"].append([boundary2_start[0], boundary2_start[1], H2])

    lane_segment_boundaries.append(segment_boundaries)

# Create LineSet for 3D boundaries
line_set_3d = o3d.geometry.LineSet()
line_set_3d.points = o3d.utility.Vector3dVector(boundary_points_flat)
line_set_3d.lines = o3d.utility.Vector2iVector(boundary_lines)

line_colors = [[1, 0, 0] for _ in range(len(boundary_lines))]  # Red
line_set_3d.colors = o3d.utility.Vector3dVector(line_colors)

# Step 2: Highlight Matched Points
light_gray = [0.827, 0.827, 0.827]  # Normalized RGB for light gray
colors = np.tile(light_gray, (len(pcd_3d_points_rotated), 1)) 

# Assign colors to the PCD
pcd_3d.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries([pcd_3d, line_set_3d, midpoints_pcd])

def convert_cartesian_to_latlon(Bx, Ly, B_origin, L_origin):
    # Calculate latitude
    B = B_origin + (Ly / 111000.0)
    # Calculate longitude
    L = L_origin + (Bx / (111320.0 * math.cos(math.radians(B_origin))))
    return B, L

# Helper function to create XML elements
def create_node_element(node_id, x, y, z):
    lat, lon = convert_cartesian_to_latlon(x, y, 37.774929, -122.419418)  # B, L for
    return ET.Element(
        "node",
        id=str(node_id),
        lat=str(lat),
        lon=str(lon),
        #z=str(z),
        version="1"
    )

def create_way_element(way_id, node_ids):
    way_element = ET.Element("way", id=str(way_id), version="1")
    for node_id in node_ids:
        nd = ET.SubElement(way_element, "nd", ref=str(node_id))
    return way_element

def create_relation_element(relation_id, left_way_id, right_way_id):
    relation = ET.Element("relation", id=str(relation_id), version="1")
    ET.SubElement(relation, "member", type="way", ref=str(left_way_id), role="left")
    ET.SubElement(relation, "member", type="way", ref=str(right_way_id), role="right")
    # Add the required type tag
    ET.SubElement(relation, "tag", k="type", v="lanelet")
    return relation

osm_root = ET.Element("osm", version="0.6")
node_id = 1
way_id = 1
relation_id = 1

for segment in lane_segment_boundaries:
    # Add centerline nodes and way
    centerline_node_ids = []
    for point in segment["centerline"]:
        node_element = create_node_element(node_id, point[0], point[1], point[2])
        osm_root.append(node_element)
        centerline_node_ids.append(node_id)
        node_id += 1
    centerline_way = create_way_element(way_id, centerline_node_ids)
    osm_root.append(centerline_way)
    centerline_way_id = way_id
    way_id += 1

    # Add left boundary nodes and way
    left_node_ids = []
    for point in segment["left_boundary"]:
        node_element = create_node_element(node_id, point[0], point[1], point[2])
        osm_root.append(node_element)
        left_node_ids.append(node_id)
        node_id += 1
    left_boundary_way = create_way_element(way_id, left_node_ids)
    osm_root.append(left_boundary_way)
    left_way_id = way_id
    way_id += 1

    # Add right boundary nodes and way
    right_node_ids = []
    for point in segment["right_boundary"]:
        node_element = create_node_element(node_id, point[0], point[1], point[2])
        osm_root.append(node_element)
        right_node_ids.append(node_id)
        node_id += 1
    right_boundary_way = create_way_element(way_id, right_node_ids)
    osm_root.append(right_boundary_way)
    right_way_id = way_id
    way_id += 1

    # Add relation tying centerline to boundaries
    relation = create_relation_element(relation_id, left_way_id, right_way_id)
    osm_root.append(relation)
    relation_id += 1

# Write to file
tree = ET.ElementTree(osm_root)
tree.write(lanelet2_file, encoding="utf-8", xml_declaration=True)
print("Written")