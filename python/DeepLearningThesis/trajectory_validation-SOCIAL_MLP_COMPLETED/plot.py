import matplotlib.pyplot as plt
import numpy as np
import cv2
import os


def plot_bar(x, y, width):
    median_x = np.median(x)
    median_y = np.median(y)
    min_x = median_x - 10
    max_x = median_x + 10
    min_y = 0
    max_y = median_y + 10
    plt.axis([min_x, max_x, min_y, max_y])
    plt.bar(x, y, width=width)
    plt.show()


def plot_trajectory(trajectory, markers, show=False):
    plt.plot(trajectory[:, 0], trajectory[:, 1], markers)
    plt.axis("equal")
    if show:
        plt.show()


def image_from_frame(frame, dataset_name, frame_id, markers, output_folder):
    plt.axis('equal')
    for agent in frame:
        plt.plot(agent[4], agent[5], markers)
    plt.savefig(output_folder + dataset_name + '/' + repr(frame_id) + '.png', dpi=300)
    plt.cla()


def video_from_dataset(dataset_name, output_folder):
    image_folder = output_folder + dataset_name + '/'
    video_name = output_folder + dataset_name + '/video.avi'
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 1, (width, height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()
