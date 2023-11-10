### Changes from F21
# Print statements added to test functionality
# Inclusion zone extended to test functionality
# Inclusion zone supressed as no values are passing through
# F22::: blue and yellow walls auto set to temp values as the walls with type 'none' return error
# modified the fit_wall function to unzip coords correctly
# meta data removed (see imports as well)


import numpy as np
import math
from .path_planner_abc import PathPlannerABC
from scipy.interpolate import UnivariateSpline
#from driver.utils.meta_helper import points_to_point_collection
from sklearn.neighbors import NearestNeighbors
import networkx as nx

# Create inclusion box on horizontal plane to define which cones to be considered
# Values changed, see F21
INCLUSION_X = [-1000, 1000]
INCLUSION_Y = [-1000, 1000]


class WallCentreLine(PathPlannerABC):
    def __init__(self):
        super().__init__()

        self.blue_wall = None
        self.yellow_wall = None

    def path_generator(self, blue_coords, yellow_coords, vehicle_vectors, parameters={}):
        """
        Generate a centre line by approximating walls on selected mapped cones
        """

        # Create inclusion box
        inclusion_box = (
            (parameters.get("inclusion_x_0", INCLUSION_X[0]), parameters.get("inclusion_x_1", INCLUSION_X[1])),
            (parameters.get("inclusion_y_0", INCLUSION_Y[0]), parameters.get("inclusion_y_1", INCLUSION_Y[1])),
        )

        if blue_coords is None or yellow_coords is None:
            return

        # select relevant cone coords for wall inclusion
        included_b = list(self.select_included_coords(blue_coords,
                                                      vehicle_vectors,
                                                      inclusion_box))

        included_y = list(self.select_included_coords(yellow_coords,
                                                      vehicle_vectors,
                                                      inclusion_box))

        ### path_planner_test
        #print('incb = ', included_b, '. incy = ', included_y)
        ###

        # Returns cone co-ordinates sorted in order of proximity
        # Although this should already be sorted through the select_included_coords function
        def sort_cones(cones):
            return sorted(cones, key=lambda c: np.linalg.norm(
                np.array(c[1]) - (vehicle_vectors[0] + vehicle_vectors[1] * inclusion_box[1][0]))
            )

        ### F22::: suppressed use of included coords
        #organised_b = self.organise_cones(sort_cones(included_b))
        #organised_y = self.organise_cones(sort_cones(included_y))
        organised_b = self.organise_cones(sort_cones(blue_coords))
        organised_y = self.organise_cones(sort_cones(yellow_coords))

        ### path_planner_test
        # print('orgb = ', organised_b, '. orgy = ', organised_y)
        ###

        # fit walls
        temp_b_wall = self.fit_wall(organised_b)
        temp_y_wall = self.fit_wall(organised_y)

        # print(organised_y)
        # print(organised_b)

        # F22::: blue and yellow walls auto set to temp values as the walls with type 'none' return error
        self.blue_wall = temp_b_wall
        self.yellow_wall = temp_y_wall

        # print('tempbwall = ', temp_b_wall, ' with length = ', len(temp_b_wall))
        # print('tempywall = ', temp_y_wall, ' with length = ', len(temp_y_wall))

        # update saved walls if appropriate
        if len(temp_b_wall) > 1:
            self.blue_wall = temp_b_wall
        if len(temp_y_wall) > 1:
            self.yellow_wall = temp_y_wall

        length = max(len(self.blue_wall), len(self.yellow_wall))

        def avg_coords(j):
            """
            Calculates the centreline between two wall curves
            """
            return [np.mean([self.blue_wall[i][j], self.yellow_wall[i][j]]) for i in range(length)]

        c_line = list(zip(avg_coords(0), avg_coords(1), avg_coords(2)))

        try:
            inc_b_coords, _ = zip(*organised_b)
            inc_y_coords, _ = zip(*organised_y)
        except ValueError:
            inc_b_coords, inc_y_coords = None, None

        # Meta data temporarily commented out
        if self.blue_wall is not None and self.yellow_wall is not None:
        #    meta = self.generate_meta_collections(self.blue_wall, self.yellow_wall,
        #                                          inc_b_coords, inc_y_coords)
            meta = None # ADDED
        else:
            meta = None

        return c_line, meta

    def fit_wall(self, coords, s=1000):
        """
        Given the center line, transform the points to be in the coordinate frame of the car
        Fit a line centered on the car onto the points
        """
        
        resolution = 20

        if len(coords) <= 1:
            points = []
            for i in range(resolution):
                points.append(coords[0])
            return points

        ### F22::: unzip the coords correctly in one line
        # xyz, weights = zip(*coords)
        # x, y, _ = zip(*xyz)
        x, y, weights = zip(*coords)

        k = 1 if len(coords) <= 2 else 2

        points = np.array([x, y]).T

        # Linear length along the line:
        distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        distance = np.insert(distance, 0, 0)/distance[-1]
        
        # Build a list of the spline function, one for each dimension:
        splines = [UnivariateSpline(distance, coords, w=weights, k=k, s=s) for coords in points.T]

        # Computed the spline for the asked distances:
        alpha = np.linspace(0, 1, resolution)

        i=0
        for spl in splines:
            i+=1
            if i==1:
                points_fitted=spl(alpha)
            else:
                points_fitted = np.vstack((points_fitted,spl(alpha)))
        points_fitted = points_fitted.T
        #points_fitted = np.vstack(spl(alpha) for spl in splines).T

        new_x, new_y = zip(*points_fitted)

        return list(zip(new_x, new_y, [0. for _ in range(resolution)]))

    def select_included_coords(self, coords, vehicle_vectors, inclusion_box):
        """
        Select the relevant coords based on distance and angle
        """
        vehicle_position, vehicle_forward, vehicle_right = vehicle_vectors
        inclusion_x, inclusion_y = inclusion_box

        def distance(A, B):
            """
            Calculate distance between point A(x,y,z) and point B(x,y,z)
            """
            return np.linalg.norm(A - B)

        def angle(A, B, f):
            """
            Calculate angle between a direction vector (f) and the vector pointing from (p) to the cone
            """
            dist = distance(A, B)
            rel = A - B
            return np.dot(f, rel) / dist

        prox_forward, prox_backward = [], []
        for i, coord in enumerate(coords):
            dist = distance(coord[0], vehicle_position)
            ang = angle(coord[0], vehicle_position, vehicle_forward)
            x, y = dist * math.cos(ang), dist * math.sin(ang)
            # Select cones either close to the vehicle, or in front of it, but further away
            if inclusion_x[0] < x < inclusion_x[1] and 0 < y < inclusion_y[1]:
                prox_forward.append([i, dist])
            if inclusion_x[0] < x < inclusion_x[1] and inclusion_y[0] < y < 0:
                prox_backward.append([i, dist])

        prox_forward = sorted(prox_forward, key=lambda x: x[1])
        prox_backward = sorted(prox_backward, key=lambda x: x[1], reverse=True)

        return [coords[idx] for idx, _ in prox_backward + prox_forward]

    def organise_cones(self, coords):
        """
        Cone ordering algorithm: REF: https://stackoverflow.com/a/37744549/4441404
        """

        if len(coords) <= 3:
            return coords

        #xyz, weights = zip(*coords)
        #x, y, z = list(map(np.array, zip(*xyz)))

        x, y, w = zip(*coords) # location (x,y) and weight (w)

        points = np.c_[x, y]

        # Changed '2' to 'n_neighbors=2' as command wasn't being recognised
        clf = NearestNeighbors(n_neighbors=2)
        clf.fit(points)
        G = clf.kneighbors_graph()

        T = nx.from_scipy_sparse_matrix(G)

        opt_order = list(nx.dfs_preorder_nodes(T, 0))

#        if len(coords) >= 3:
#            remove_iter = 5
#
#            def remove_in_window(order, window_size, n=2):
#                """
#                Create a window of window_size points.
#                If the sum of distances between all points is n times greater
#                than the distance of the first and last points
#                mark all points inbetween to be removed
#                """
#                window = list(zip(*[order[i:len(order) - window_size + i + 1] for i in range(window_size)]))
#                return [i for t in filter(
#                    lambda t:
#                        np.linalg.norm(t[1][0] - t[1][-1]) * n <
#                        sum([np.linalg.norm(t[1][i] - t[1][i + 1]) for i in range(len(t[1]) - 1)]),
#                    [(t[1:-1], [np.array([x[i], y[i]]) for i in t]) for t in window]
#                ) for i in t[0]]
#
#            def remove_end_of_window(order, m=3):
#                """
#                Create a window of 3 size
#                If the second two points are m times further away than the first two,
#                mark the last point to be removed
#                """
#                window = list(zip(opt_order[:-2], opt_order[1:-1], opt_order[2:]))
#
#                return list(map(lambda r: r[0], filter(
#                    lambda t: np.linalg.norm(t[1] - t[2]) * m < (np.linalg.norm(t[2] - t[3])),
#                    [(t[2], *[np.array([x[i], y[i]]) for i in t]) for t in window]
#                )))
#
#            def remove_from_opt(to_remove):
#                for i in to_remove:
#                    opt_order.remove(i)
#
#            for _ in range(remove_iter):
#                remove_from_opt(remove_in_window(opt_order, 3))
#                remove_from_opt(remove_in_window(opt_order, 4))
#                remove_from_opt(remove_end_of_window(opt_order))
        
        # F22::: data type doesnt match, new method added
        #xx = x[opt_order]
        #yy = y[opt_order]

        # initialise ordered arrays
        x_ordered = [0 for i in range(len(opt_order))]
        y_ordered = [0 for i in range(len(opt_order))]
        w_ordered = [0 for i in range(len(opt_order))]

        # re-create the coordinates in the new order
        for [i, j] in zip(opt_order, range(len(opt_order))):
            x_ordered[j] = x[i]
            y_ordered[j] = y[i]
            w_ordered[j] = w[i]

        return list(zip(x_ordered, y_ordered, w_ordered))

    def generate_meta_collections(self, blue, yellow, included_b, included_y):
        return [points_to_point_collection(blue, 'blue_wall', 'blue', 'line'),
                points_to_point_collection(yellow, 'yellow_wall', 'yellow', 'line'),
                points_to_point_collection(included_b, 'included_blue', 'purple', 'point'),
                points_to_point_collection(included_y, 'included_yellow', 'purple', 'point')]
