import numpy as np
from scipy.interpolate import UnivariateSpline
from sklearn.metrics import r2_score
from sklearn.linear_model import LinearRegression
from util import load_pickle, write_pickle
from plot import plot_bar, plot_trajectory, image_from_frame, video_from_dataset
import matplotlib.pyplot as plt

state_plane_code_dict = {
    'us101': 405,
    'lankershim': 405,
    'peachtree': 1002,
    'i80': 403
}

extremes_dict = {
    'us101': [(1966310.5193855157, 34.1396026), (1966818.0568763716, 34.1355469), (570556.8745427944, -118.3598137),
              (571017.4442209217, -118.3651908)]
}


class Statistics:
    def __init__(self, data_file):

        # Load data from the pickled file
        try:
            self.raw_data = load_pickle(data_file)
        except OSError:
            raise

        # Get all the data from the pickle file
        self.data = self.raw_data[0]
        self.trajectories = {}

    def csv_from_frame_array(self, state_plane_code, metric='m'):
        pass

    def video_from_frame_array(self, output_path, dataset_name):
        for frame_id, frame in enumerate(self.data):
            actual_frame = frame[~np.all(frame == 0, axis=1)]
            image_from_frame(actual_frame, dataset_name, frame_id, 'bo', output_path)
        # video_from_dataset(dataset_name, output_path)

    def create_array(self, output_path=None):
        for id, dataset in enumerate(self.data):
            if id not in self.trajectories:
                self.trajectories[id] = {}
            for frame in self.data:
                for agent in frame:
                    if len(agent) == 3:
                        if agent[0] in self.trajectories[id]:
                            self.trajectories[id][agent[0]].append(agent[1:])
                        else:
                            self.trajectories[id][agent[0]] = [agent[1:]]
                    elif len(agent) == 19:
                        if (agent[0], agent[1], agent[6], agent[7], agent[8]) in self.trajectories[id]:
                            data = np.concatenate([agent[2:6], agent[9:len(agent)]])
                            self.trajectories[id][(agent[0], agent[1], agent[6], agent[7], agent[8])].append(data)
                        elif (agent[0], agent[1], agent[6], agent[7], agent[8]) != (0.0, 0.0, 0.0, 0.0, 0.0):
                            data = np.concatenate([agent[2:6], agent[9:len(agent)]])
                            self.trajectories[id][(agent[0], agent[1], agent[6], agent[7], agent[8])] = [data]
        if output_path is not None:
            try:
                write_pickle(output_path, self.trajectories)
            except OSError:
                raise
        return self.trajectories

    def load_array(self, array_path):
        try:
            self.trajectories = load_pickle(array_path)
        except OSError:
            raise

    def sanitize_array(self, output_path=None):
        for dataset in self.trajectories:
            for agent in self.trajectories[dataset].keys():
                agent_trajectory = self.trajectories[dataset][agent]
                coords = np.vstack(agent_trajectory)
                temp = coords[coords[:, -1].argsort()]
                unique_indices = np.unique(temp, return_index=True, axis=0)[1]
                unique_trajectory = np.array([temp[index] for index in sorted(unique_indices)])
                d = np.diff(coords, axis=0)[:, -1]
                negative_indices = np.where(d < 0)
                if len(d[negative_indices]) > 0:
                    pass
                self.trajectories[dataset][agent] = list(unique_trajectory)
        if output_path is not None:
            try:
                write_pickle(output_path, self.trajectories)
            except OSError:
                raise
        return self.trajectories

    def export_extremes(self, state_plane_code, metric='m', output_path=None):
        agent_trajectories = [np.vstack(traj)[:, 2:4] for traj in list(self.trajectories[0].values())]
        agent_extremes = np.vstack([np.array([np.min(agent_trajectory[:, 0]), np.max(agent_trajectory[:, 0]),
                                              np.min(agent_trajectory[:, 1]), np.max(agent_trajectory[:, 1])])
                                    for agent_trajectory in agent_trajectories])
        # min_x_point = agent_trajectories[np.argmin(agent_extremes[:, 0])][agent_trajectories[np.argmin(agent_extremes[:,0])][:, 0] == np.min(agent_extremes[:, 0])][0]
        # max_x_point = agent_trajectories[np.argmax(agent_extremes[:, 1])][agent_trajectories[np.argmax(agent_extremes[:,1])][:, 0] == np.max(agent_extremes[:, 1])][0]
        # min_y_point = agent_trajectories[np.argmin(agent_extremes[:, 2])][agent_trajectories[np.argmin(agent_extremes[:,2])][:, 1] == np.min(agent_extremes[:, 2])][0]
        # max_y_point = agent_trajectories[np.argmax(agent_extremes[:, 3])][agent_trajectories[np.argmax(agent_extremes[:,3])][:, 1] == np.max(agent_extremes[:, 3])][0]
        min_x = np.min(agent_extremes[:, 0])
        max_x = np.max(agent_extremes[:, 1])
        min_y = np.min(agent_extremes[:, 2])
        max_y = np.max(agent_extremes[:, 3])
        down_left = [min_x, min_y]
        down_right = [max_x, min_y]
        up_left = [min_x, max_y]
        up_right = [max_x, max_y]
        csv_point_list = ['Position', down_left, down_right, up_left, up_right]
        max_rows = 10000
        sum_rows = 0
        csv_point_list = ['Position']
        csv_agent_point = ['Agent']
        csv_counter = 0
        for i, agent_trajectory in enumerate(agent_trajectories):
            if sum_rows + agent_trajectory.shape[0] <= max_rows:
                sum_rows += agent_trajectory.shape[0]
                for point in agent_trajectory:
                    csv_point_list.append(repr(state_plane_code_dict[state_plane_code]) + ' ' + repr(point[0]) + metric
                                          + ' ' + repr(point[1]) + metric)
                    csv_agent_point.append(repr(i))
            else:
                np.savetxt(output_path+'_'+repr(csv_counter)+'_points.csv', np.array(csv_point_list), delimiter=',',
                           fmt='%s')
                np.savetxt(output_path+'_'+repr(csv_counter)+'_agents.csv', np.array(csv_agent_point), delimiter=',',
                           fmt='%s')
                csv_counter += 1
                sum_rows = 0
                csv_point_list = ['Position']
                csv_agent_point = ['Agent']
                sum_rows += agent_trajectory.shape[0]
                for point in agent_trajectory:
                    csv_point_list.append(repr(state_plane_code_dict[state_plane_code]) + ' ' + repr(point[0]) + metric
                                          + ' ' + repr(point[1]) + metric)
                    csv_agent_point.append(repr(i))
        # for i, extreme in enumerate(csv_point_list):
        #    if i == 0:
        #        continue
        #   csv_point_list[i] = repr(state_plane_code_dict[state_plane_code]) + ' ' + repr(csv_point_list[i][0]) + metric + ' ' + \
        #                  repr(csv_point_list[i][1]) + metric
        # if output_path is not None:
        #    np.savetxt(output_path, np.array(csv_point_list), delimiter=',', fmt='%s')
        return csv_point_list

    def transform_to_wgs84(self, dataset_name):
        extremes = extremes_dict[dataset_name]
        extremes_x = [extremes[0][0], extremes[1][0]]
        extremes_y = [extremes[2][0], extremes[3][0]]
        lat = [extremes[0][1], extremes[1][1]]
        long = [extremes[2][1], extremes[3][1]]
        wgs84_trajectories = [np.hstack([np.interp(np.vstack(traj)[:, 2], extremes_x, lat).reshape(-1, 1),
                                         np.interp(np.vstack(traj)[:, 3], extremes_y, long).reshape(-1, 1)]) for traj in
                              list(self.trajectories[0].values())[0:3]]
        return wgs84_trajectories

    def offset(self):
        offsets_x = {}
        offsets_y = {}
        previous_x = 0
        previous_y = 0
        flag = 0
        for dataset in self.trajectories:
            for agent in self.trajectories[dataset]:
                for position in self.trajectories[dataset][agent]:
                    if flag == 0:
                        previous_x = position[0]
                        previous_y = position[1]
                        flag = 1
                    else:
                        d_x = position[0] - previous_x
                        d_x = float("{0:.2f}".format(d_x))
                        d_y = position[1] - previous_y
                        d_y = float("{0:.2f}".format(d_y))
                        if d_x in offsets_x:
                            offsets_x[d_x] += 1
                        else:
                            offsets_x[d_x] = 1
                        if d_y in offsets_y:
                            offsets_y[d_y] += 1
                        else:
                            offsets_y[d_y] = 1
                        previous_x = position[0]
                        previous_y = position[1]
                flag = 0
                previous_y = 0
                previous_x = 0
        print(offsets_x)
        print(offsets_y)
        total = sum(offsets_x.values(), 0.0)
        print(total)
        # total_y = sum(offsets_y.values(), 0.0)
        offsets_y = {k: v / total for k, v in offsets_y.items()}
        offsets_x = {k: v / total for k, v in offsets_x.items()}
        offsets_x_keys = list(offsets_x.keys())
        offsets_x_values = list(offsets_x.values())
        offsets_y_keys = list(offsets_y.keys())
        offsets_y_values = list(offsets_y.values())
        print(sum(offsets_y.values()))
        print(sum(offsets_x.values()))
        print(len(offsets_x_keys))
        print(len(offsets_y_keys))
        print(sum(offsets_x.values()))
        print(sum(offsets_y.values()))
        plot_bar(offsets_x_keys[::3], offsets_x_values[::3], 0.1)
        plot_bar(offsets_y_keys[::3], offsets_y_values[::3], 0.1)

    def curvature(self):
        curvatures = {}
        for dataset in self.trajectories.values():
            for agent_trajectory in dataset.values():
                coords = np.vstack(agent_trajectory)[::2, :]
                if coords.shape[0] > 1:
                    curvature_vector = self.curvature_from_trajectory(coords)
                    for curvature in curvature_vector:
                        curvature = float('{0:.3f}'.format(curvature))
                        if curvature not in curvatures.keys():
                            curvatures[curvature] = 1
                        else:
                            curvatures[curvature] += 1
        curvatures = {k: v for k, v in curvatures.items() if not np.isnan(k)}
        curvatures = {k: v / sum(curvatures.values()) for k, v in curvatures.items()}
        print(sum(curvatures.values()))
        plot_bar(curvatures.keys(), curvatures.values(), 0.1)

    @staticmethod
    def curvature_from_trajectory(trajectory):
        x_grad = np.gradient(trajectory[:, 0])
        y_grad = np.gradient(trajectory[:, 1])
        x_d_grad = np.gradient(x_grad)
        y_d_grad = np.gradient(y_grad)
        return (x_grad * y_d_grad - y_grad * x_d_grad) / np.power((np.power(x_grad, 2) + np.power(y_grad, 2)), 3 / 2)

    @staticmethod
    def space_subsample_trajectory(trajectory, limit=0.3):
        d = np.diff(trajectory, axis=0)[:, 0:2]
        segdists = np.sqrt((d ** 2).sum(axis=1))
        del_idx = []
        dist_sum = 0
        prev_idx = 0
        for i, dist in enumerate(segdists):
            dist_sum += dist
            if dist_sum >= limit:
                dist_sum = 0
                del_idx += list(range(prev_idx + 1, i))
                prev_idx = i
        return np.delete(trajectory, del_idx, axis=0)

    @staticmethod
    def time_subsample_trajectory(trajectory, limit=1):
        d = np.diff(trajectory, axis=0)[:, -1]
        del_idx = []
        diff_sum = 0
        prev_idx = 0
        for i, diff in enumerate(d):
            diff_sum += diff
            if diff_sum >= limit:
                diff_sum = 0
                del_idx += list(range(prev_idx + 1, i))
                prev_idx = i
        return np.delete(trajectory, del_idx, axis=0)

    def curvature_with_space_sampling(self):
        curvatures = {}
        degree = 4
        for dataset in self.trajectories.values():
            for agent_trajectory in dataset.values():
                coords = np.vstack(agent_trajectory)
                smooth_trajectory, x_fit, y_fit = self.smooth_trajectory(coords, degree)
                if smooth_trajectory is not None:
                    sampled_trajectory = self.space_subsample_trajectory(smooth_trajectory)
                    curvature_vector = self.curvature_from_trajectory(sampled_trajectory)
                    for curvature in curvature_vector:
                        curvature = float('{0:.2f}'.format(curvature))
                        if curvature not in curvatures.keys():
                            curvatures[curvature] = 1
                        else:
                            curvatures[curvature] += 1
        curvatures = {k: v for k, v in curvatures.items() if not np.isnan(k)}
        curvatures = {k: v / sum(curvatures.values()) for k, v in curvatures.items()}
        print(sum(curvatures.values()))
        plot_bar(list(curvatures.keys()), list(curvatures.values()), 0.005)

    def curvature_with_time_sampling(self):
        curvatures = {}
        degree = 4
        for dataset in self.trajectories.keys():
            for agent in self.trajectories[dataset].keys():
                agent_trajectory = self.trajectories[dataset][agent]
                coords = np.vstack(agent_trajectory)
                smooth_trajectory, x_fit, y_fit = self.smooth_trajectory(coords, degree)
                if smooth_trajectory is not None:
                    sampled_trajectory = self.time_subsample_trajectory(smooth_trajectory)
                    curvature_vector = self.curvature_from_trajectory(sampled_trajectory)
                    for curvature in curvature_vector:
                        curvature = float('{0:.2f}'.format(curvature))
                        if curvature not in curvatures.keys():
                            curvatures[curvature] = 1
                        else:
                            curvatures[curvature] += 1
        curvatures = {k: v for k, v in curvatures.items() if not np.isnan(k)}
        curvatures = {k: v / sum(curvatures.values()) for k, v in curvatures.items()}
        print(sum(curvatures.values()))
        plot_bar(list(curvatures.keys()), list(curvatures.values()), 0.005)

    def noise(self):
        degree = 4
        diff_x = []
        diff_y = []
        for dataset in self.trajectories.values():
            for agent_trajectory in dataset.values():
                coords = np.vstack(agent_trajectory)
                unique_indices = np.unique(coords, return_index=True, axis=0)[1]
                unique_trajectory = np.array([coords[index] for index in sorted(unique_indices)])
                smooth_trajectory, x_fit, y_fit = self.smooth_trajectory(coords, degree)
                if smooth_trajectory is not None:
                    sum_diff_x = np.sum(np.abs(unique_trajectory[:, 0] - x_fit)) / unique_trajectory.shape[0]
                    sum_diff_y = np.sum(np.abs(unique_trajectory[:, 1] - y_fit)) / unique_trajectory.shape[0]
                    diff_x.append(sum_diff_x)
                    diff_y.append(sum_diff_y)
        print("standard deviation in x:", np.std(diff_x))
        print("standard deviation in y:", np.std(diff_y))

    @staticmethod
    def smooth_trajectory(trajectory, degree):
        if trajectory.shape[0] > degree + 3:
            # t = range(trajectory.shape[0])
            x = trajectory[:, 0]
            y = trajectory[:, 1]
            t = trajectory[:, -1]
            spl_x = UnivariateSpline(t, x, k=degree)
            spl_y = UnivariateSpline(t, y, k=degree)
            sampling = np.linspace(t[0], t[-1], 1000)
            smooth_trajectory = np.hstack([spl_x(sampling).reshape(-1, 1), spl_y(sampling).reshape(-1, 1),
                                           sampling.reshape(-1, 1)])
            x_fit = spl_x(t)
            y_fit = spl_y(t)
        else:
            smooth_trajectory = None
            x_fit = None
            y_fit = None
        return smooth_trajectory, x_fit, y_fit

    def transform_global_to_previous_local(self, output_path=None):
        dataset_counter = 0
        for dataset in self.trajectories:
            agent_counter = 0
            print("starting dataset", dataset_counter)
            for agent in self.trajectories[dataset]:
                if agent_counter % 100 == 0:
                    print(agent_counter, " out of ", len(self.trajectories[dataset]), " completed")
                if len(self.trajectories[dataset][agent]) > 1:
                    self.trajectories[dataset][agent] = self.transform_trajectory_points(
                        self.trajectories[dataset][agent])
                    for idx in range(1, len(self.trajectories[dataset][agent]) - 1):
                        self.trajectories[dataset][agent][idx + 1] = self.transform_trajectory_points(
                            self.trajectories[dataset][agent][idx:])[1:]
                agent_counter += 1
            dataset_counter += 1
        if output_path is not None:
            try:
                write_pickle(output_path, self.trajectories)
            except OSError:
                raise

    def transform_trajectory_points(self, trajectory):
        trajectory_points = list(np.vstack(trajectory)[:, :2])
        plot_trajectory(np.vstack(trajectory_points), 'bo')
        transformed_trajectory = np.vstack(self.trans_global_local(trajectory_points))
        plot_trajectory(transformed_trajectory, 'go', show=True)
        trajectory_additional_data = np.vstack(trajectory)[:, 2:]
        transformed_trajectory = np.hstack((transformed_trajectory, trajectory_additional_data))
        return list(transformed_trajectory)

    @staticmethod
    def trans_global_local(trajectory):
        trajectory = np.array(trajectory)
        # print(trajectory.shape)
        # print(trajectory)
        first_orientation_x = trajectory[1][0] - trajectory[0][0]
        first_orientation_y = trajectory[1][1] - trajectory[0][1]
        orientation_x = np.average(np.diff(trajectory[:, 0]))
        orientation_y = np.average(np.diff(trajectory[:, 1]))
        # print(first_orientation_x, first_orientation_y)
        theta = np.arctan2([orientation_y], [orientation_x])
        # print(theta)
        cos = np.cos(theta)[0]
        sin = np.sin(theta)[0]
        trans_matrix = np.array([[cos, -sin, trajectory[0][0]], [sin, cos, trajectory[0][1]], [0, 0, 1]])
        inv_trans_matrix = np.linalg.inv(trans_matrix)
        trajectory = trajectory.transpose()
        # print(trajectory.shape)
        homogeneous = np.ones((1, trajectory.shape[1]))
        trajectory = np.vstack((trajectory, homogeneous))
        # print(trajectory.shape)
        trans_trajectory = inv_trans_matrix.dot(trajectory).transpose()
        trans_trajectory = trans_trajectory[:, :-1]
        # plot_trajectory(trans_trajectory, 'bo', show=True)

        return list(trans_trajectory)

    def trans_agent_location(self, output_path=None):
        for dataset in self.trajectories:
            for agent in self.trajectories[dataset]:
                if len(self.trajectories[dataset][agent]) > 1:
                    self.trajectories[dataset][agent] = self.transform_trajectory_points(
                        self.trajectories[dataset][agent])
        if output_path is not None:
            try:
                write_pickle(output_path, self.trajectories)
            except OSError:
                raise

    def r_squared(self):
        r_squared = []
        r_squared_y = []
        prob_x = []
        prob_y = []
        for dataset in self.trajectories:
            for agent in self.trajectories[dataset]:
                t_coords = self.trans_global_local(self.trajectories[dataset][agent])
                coords = np.vstack(t_coords)
                x = coords[:, 0]
                y = coords[:, 1]
                # r_squared for x values
                ones = np.ones(x.shape)
                X = np.vstack((ones, x)).transpose()
                reg = LinearRegression().fit(X, y)
                pred = reg.predict(X)
                score_x = r2_score(y, pred)
                score_x = float("{0:.2f}".format(score_x))
                r_squared.append(score_x)
                # plt.plot(x, y, 'g', x, pred, '-')
                # plt.show()

            r_squared_unique = np.unique(r_squared)
            total = len(r_squared)
            for r in r_squared_unique:
                prob_y.append(r_squared.count(r) / total)
            print(r_squared)
            print(prob_y)
            plot_bar(r_squared_unique, prob_y, 0.02)


