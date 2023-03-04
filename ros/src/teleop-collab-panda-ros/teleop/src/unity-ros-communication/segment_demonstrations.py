#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse
from teleop_msgs.srv import ReturnOverlaps, ReturnOverlapsResponse
from teleop_msgs.srv import TrajToSegments, TrajToSegmentsResponse

import numpy as np
from numpy.core.numeric import load
from sklearn import mixture
from sklearn.mixture.gaussian_mixture import GaussianMixture
from scipy import stats
from sklearn.decomposition import PCA

import matplotlib.pyplot as plt

"""
Class for segmenting a multi-DOF robot trajectory into components.
"""


class SegmentDemonstration:
    def __init__(self):
        self.demo_node = rospy.init_node("segment_demonstration_server")
        self.demo_arr = None
        self.arm_joint_ind = 2
        # Service called externally to segment demonstration
        self.demo_service = rospy.Service("segment_demonstration_service", TrajToSegments,
                                          self.load_and_perform_segmentation)
        self.data_freq = 25  # Hz
        print("Ready to segment")

        rospy.spin()

    # Load demonstration (currently from file)
    def load(self, filename):
        loaded_demonstration = np.loadtxt(filename)
        return loaded_demonstration

    # Pre-process the data into the right structure
    def pre_process(self, loaded_demonstration):
        # Delete first two rows (gripper dimensions not used here for now)
        loaded_demonstration = np.delete(loaded_demonstration, [0, 1], axis=1)
        # print(self.loaded_demonstration.shape)

        # Extract originalt trajectory w/o temporality (time)
        loaded_demonstration_original = np.delete(loaded_demonstration, loaded_demonstration.shape[1] - 1, axis=1)
        loaded_demonstration_original = loaded_demonstration_original.T
        return loaded_demonstration_original

    # Reduce dims of the input
    def pca_module(self, demo, num_components):
        pca = PCA(n_components=0.99, svd_solver="full")
        demo = demo.T
        pca.fit(demo)
        reduced_demo = pca.transform(demo)
        print("Found " + str(len(pca.components_)) + " components")
        return reduced_demo.T, pca.explained_variance_, pca.components_

    # Apply GMM model
    def gmm_module(self, data):
        n_gmm_range = range(1, 10)
        lowest_bic = np.infty
        bic_list = []
        best_gmm = None
        best_gmm_ind = 0

        for n_gmm in n_gmm_range:
            gmm = mixture.GaussianMixture(n_components=n_gmm, init_params='kmeans')
            gmm.fit(data)
            bic_list.append(gmm.bic(data))
            if bic_list[-1] < lowest_bic:
                lowest_bic = bic_list[-1]
                best_gmm = gmm
                best_gmm_ind = n_gmm
                print(lowest_bic)
                print(n_gmm)
                # print("New best GMM")

        print(bic_list)

        return best_gmm

    # Split up the temporal and spatial means/covariance matrix
    def divide_spatial_temporal(self, best_gmm):
        component_means = []
        components_cov = []
        for component in range(best_gmm.n_components):
            means_shape = best_gmm.means_[component].shape
            means_temporal = best_gmm.means_[component][-1].reshape(1, 1)
            means_spatial = best_gmm.means_[component][:-1].reshape(1, means_shape[0] - 1)
            final_means = np.concatenate((means_temporal, means_spatial), axis=1)

            cov_shape = best_gmm.covariances_[component].shape
            cov_spatial = best_gmm.covariances_[component][:-1, :-1]
            cov_temporal = best_gmm.covariances_[component][-1, -1]
            cov_temporal = np.array(cov_temporal).reshape(1, 1)
            cov_spatial_temporal = best_gmm.covariances_[component][:-1, -1].reshape(cov_shape[0] - 1, 1)
            cov_temporal_spatial = best_gmm.covariances_[component][-1, :-1].reshape(1, cov_shape[1] - 1)

            # Concatenate top half first
            final_cov_top = np.concatenate((cov_temporal, cov_temporal_spatial), axis=1)
            final_cov_bottom = np.concatenate((cov_spatial_temporal, cov_spatial), axis=1)
            final_cov = np.concatenate((final_cov_top, final_cov_bottom), axis=0)
            # print(final_cov)
            # print(final_cov)

            component_means.append(final_means)
            components_cov.append(final_cov)

        return component_means, components_cov

    # Get the different segments for each Gaussian
    def get_segments(self, demo, best_gmm, mean, cov):
        # print(best_gmm.weights_)
        # Contains separate terms for each gaussian
        distributions = []

        # Create the distributions
        for component in range(best_gmm.n_components):
            # dist = np.random.normal(loc=mean[component][0], scale=cov[component][0][0])
            # print('Mean, ', mean[component][0][0])
            # print('Stdev, ', cov[component][0][0])
            dist = stats.norm(mean[component][0][0], cov[component][0][0])
            distributions.append(dist)

        # Split up the segments
        h_all_components = self.get_weights_split_up(best_gmm, demo, distributions)

        # Compute overlaps
        overlaps = []
        THRESH = 0.1
        time = 0
        overlap_found = False
        while time < len(demo[-1]):
            # print(time)
            for i in range(len(h_all_components)):
                if i + 1 <= len(h_all_components):
                    for j in range(len(h_all_components)):
                        if i != j:
                            if np.isclose(h_all_components[i][time], h_all_components[j][time], atol=0.1) and \
                                    h_all_components[i][time] > THRESH and h_all_components[j][time] > THRESH:
                                overlap_found = True
                                overlaps.append(demo[-1][time])
                                break

                if overlap_found == True:
                    break

            # Increment
            if overlap_found == True:
                if time + self.data_freq <= len(demo[-1]):
                    time += self.data_freq
                    overlap_found = False
                    # print('Gap ', time)
                else:
                    break

            else:
                time += 1

        # If we find any overlaps then display them on a plot and print
        if len(overlaps) > 0:
            # Plot the DOF and h(t)
            print(overlaps)
            # fig, ax = plt.subplots(8, 1)
            # self.plot_weights_and_demo(h_all_components, demo, ax)
            # self.plot_seg(ax, overlaps)
            # plt.show()

        return overlaps

    # Plot DOF of robot and the h(t) weighting function
    def get_weights_split_up(self, best_gmm, demo, distributions):
        # Loop through time and calculate the h(t) for each Gaussian
        # TODO: Implementation is inefficient (2 loops)
        h_all_components = []
        for component in range(best_gmm.n_components):
            h_component = []
            for t in range(len(demo[-1])):
                num = best_gmm.weights_[component] * distributions[component].pdf(demo[-1][t])
                denom = self.get_denom_h(demo[-1][t], best_gmm, distributions)
                h = num / denom
                h_component.append(h)
            h_all_components.append(np.asarray(h_component))

        return h_all_components

    # Plot the weight Gaussians and the demo from all DOF
    def plot_weights_and_demo(self, h_all_components, demo, ax):
        # Plot the Gaussians
        for h in range(len(h_all_components)):
            ax[0].scatter(demo[-1], h_all_components[h])

        # Plot the joint angles
        demo_red = np.delete(demo, [0, 1], 0)  # Reduce demo so that we can plot it easily
        print(demo_red.shape)
        for d in range(len(demo_red) - 1):
            ax[d + 1].plot(demo_red[-1], demo_red[d])

    # Plot vertical lines showing each "segment"
    def plot_seg(self, ax, overlaps):
        ax[0].vlines(x=overlaps, ymin=0.05, ymax=0.95)

    def get_denom_h(self, time, best_gmm, dist_list):
        denom = 0
        # print(t)
        for component in range(best_gmm.n_components):
            # print(t)
            denom += best_gmm.weights_[component] * dist_list[component].pdf(time)
            # print(denom)
        return denom

    ###### ROS-related
    def convert_traj_to_demo(self, initial_traj):
        self.demo_arr = np.zeros((len(initial_traj.points), len(initial_traj.points[0].positions) + 1))

        demo_start_time = 0
        for i in range(len(initial_traj.points)):
            for j in range(self.arm_joint_ind, len(initial_traj.points[i].positions) + 1):
                # print(j)
                if j <= len(initial_traj.points[i].positions) - 1:
                    self.demo_arr[i, j] = initial_traj.points[i].positions[j]
                else:
                    original_time = initial_traj.points[i].time_from_start.secs + \
                                    initial_traj.points[i].time_from_start.nsecs * pow(10, -9)

                    if (i == 0):
                        demo_start_time = original_time
                        actual_time = 0
                    else:
                        actual_time = original_time - demo_start_time

                    self.demo_arr[i, j] = actual_time

    # Takes ROS service request and performs segmentation accordingly
    def load_and_perform_segmentation(self, request):
        # print('Request: ', request.data)

        # Convert trajectory into usable format first:
        self.convert_traj_to_demo(request.initial_traj)

        # initial_demo = self.load(rospkg.RosPack().get_path('teleop') + '/demos/demo.txt')
        initial_demo_modified = self.pre_process(self.demo_arr)

        # Get the PCA
        demo_after_pca, _, _ = self.pca_module(initial_demo_modified, 2)
        # append the time column back to the reduced data
        # print(demo_after_pca.shape)
        # print(initial_demo[-1, :])
        # print((np.reshape(initial_demo[-1, :], (1, initial_demo[-1, :].shape[0])).shape))
        # print(initial_demo[-1, :])

        # Transpose initial demo
        initial_demo = self.demo_arr.T

        # Append the PCA result with the time dimension from the initial demonstration
        demo_after_pca = np.append(demo_after_pca, np.reshape(initial_demo[-1, :], (1, initial_demo[-1, :].shape[0])),
                                   axis=0)

        # Get best GMM through BIC
        best_gmm = self.gmm_module(demo_after_pca.T)
        # best_gmm = self.gmm_module(initial_demo_modified.T)

        # Get final segments
        components_means, components_cov = self.divide_spatial_temporal(best_gmm)
        # print(components_cov)
        # print(components_cov.shape)
        segs = self.get_segments(initial_demo, best_gmm, components_means, components_cov)

        response = TrajToSegmentsResponse()
        overlaps_n = []
        for i in range(len(segs)):
            new = Float32()
            new.data = segs[i]
            overlaps_n.append(new)

        response.overlaps = overlaps_n
        return response


def main():
    SegmentDemonstration()


if __name__ == '__main__':
    main()
