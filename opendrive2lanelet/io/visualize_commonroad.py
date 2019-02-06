# -*- coding: utf-8 -*-

import sys
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object


def main():
    """Short helper file to visualize an xml file
    as a command line tool.

    Args:

    Returns:

    """
    if len(sys.argv) == 1 or sys.argv[1] == "--help":
        print(
            """Usage: convert.py input_file output_name.
        If no output_name is specified, output_file has name of input_file."""
        )
        exit(0)

    filename = sys.argv[1]

    scenario, _ = CommonRoadFileReader(filename).open()

    # temporary fix to get a plotable view of the scenario
    plot_center = scenario.lanelet_network.lanelets[0].left_vertices[0]
    plt.style.use("classic")
    plt.figure(figsize=(10, 10))
    plt.gca().axis("equal")
    plot_displacement_x = plot_displacement_y = 200
    plot_limits = [
        plot_center[0] - plot_displacement_x,
        plot_center[0] + plot_displacement_x,
        plot_center[1] - plot_displacement_y,
        plot_center[1] + plot_displacement_y,
    ]
    draw_object(scenario, plot_limits=plot_limits)
    plt.show()


if __name__ == "__main":
    main()
