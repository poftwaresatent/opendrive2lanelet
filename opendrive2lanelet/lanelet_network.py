# -*- coding: utf-8 -*-

"""Module to enhance LaneletNetwork class
so it can be used for conversion from the opendrive format."""

from queue import Queue
import numpy as np

from commonroad.scenario.lanelet import LaneletNetwork
from opendrive2lanelet.lanelet import ConversionLanelet

__author__ = "Benjamin Orthen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.0"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

optimal_join_split_factor = 15


class ConversionLaneletNetwork(LaneletNetwork):
    """Add functions to LaneletNetwork which
    further enable it to modify its Lanelets."""

    def remove_lanelet(self, lanelet_id: str):
        """Remove a lanelets with the specific lanelet_id
        from the _lanelets dict.

        Args:
          lanelet_id: id of lanelet to be removed.

        Returns:
          None

        """
        del self._lanelets[lanelet_id]

    def find_lanelet_by_id(self, lanelet_id) -> ConversionLanelet:
        """Find a lanelet for a given lanelet_id.
        Disable natural number check of parent class.

        Args:
          lanelet_id: The id of the lanelet to find

        Returns:
          The lanelet object if the id exists and None otherwise

        """
        return self._lanelets.get(lanelet_id)

    def prune_network(self):
        """Remove references in predecessor, successor etc. to
        non existing lanelets.

        Args:

        Returns:

        """
        self._delete_zero_width_parametric_lanes()

        lanelet_ids = [x.lanelet_id for x in self.lanelets]

        for lanelet in self.lanelets:
            lanelet.predecessor[:] = [
                pred for pred in lanelet.predecessor if pred in lanelet_ids
            ]
            lanelet.successor[:] = [
                succ for succ in lanelet.successor if succ in lanelet_ids
            ]
            if lanelet.adj_left not in lanelet_ids:
                lanelet.adj_left = None
            if lanelet.adj_right not in lanelet_ids:
                lanelet.adj_right = None

    def _delete_zero_width_parametric_lanes(self):
        """Remove all ParametricLaneGroup which have zero width at every point from
        this network.
        """
        for lanelet in self.lanelets:
            if lanelet.has_zero_width_everywhere():
                if lanelet.adj_right:
                    adj_right = self.find_lanelet_by_id(lanelet.adj_right)
                    if adj_right:
                        adj_right.adj_left = lanelet.adj_left

                if lanelet.adj_left:
                    adj_left = self.find_lanelet_by_id(lanelet.adj_left)
                    if adj_left:
                        adj_left.adj_right = lanelet.adj_right

                self.remove_lanelet(lanelet.lanelet_id)

    def update_lanelet_id_references(self, old_id: str, new_id: str):
        """Update all references to the old lanelet_id with the new_lanelet_id.

        Args:
          old_id: Old lanelet_id which has changed.
          new_id: New lanelet_id the old_id has changed into.

        """

        for lanelet in self.lanelets:
            lanelet.predecessor[:] = [
                new_id if pred == old_id else pred for pred in lanelet.predecessor
            ]

            lanelet.successor[:] = [
                new_id if succ == old_id else succ for succ in lanelet.successor
            ]

            if lanelet.adj_right == old_id:
                lanelet.adj_right = new_id

            if lanelet.adj_left == old_id:
                lanelet.adj_left = new_id

    def concatenate_possible_lanelets(self):
        """Iterate trough lanelets in network and concatenate possible lanelets together.

        Check for each lanelet if it can be concatenated with its successor and
        if its neighbors can be concatenated as well. If yes, do the concatenation.

        """
        concatenate_lanelets = []
        for lanelet in self.lanelets:

            if lanelet.adj_right is None:
                possible_concat_lanes = self.check_concatenation_potential(
                    lanelet, "left"
                )
            elif lanelet.adj_left is None:
                possible_concat_lanes = self.check_concatenation_potential(
                    lanelet, "right"
                )
            if possible_concat_lanes:
                concatenate_lanelets.append(possible_concat_lanes)

        # this dict saves the lanelet_ids which point to another lanelet_id
        # because they were concatenated together and the resulting lanelet
        # can only have one lanelet_id.
        # key in dict has been renamed to value
        replacement_ids = dict()

        for possible_concat_lanes in concatenate_lanelets:
            # prevent chains of more than one lanelet being renamed
            replacement_ids = {
                k: replacement_ids.get(v, v) for k, v in replacement_ids.items()
            }

            possible_concat_lanes = [
                (
                    replacement_ids.get(pair[0], pair[0]),
                    replacement_ids.get(pair[1], pair[1]),
                )
                for pair in possible_concat_lanes
            ]
            replacement_ids.update(
                self._concatenate_lanelet_pairs_group(possible_concat_lanes)
            )

    def _concatenate_lanelet_pairs_group(self, lanelet_pairs: list) -> dict:
        """Concatenate a group of lanelet_pairs, with setting correctly the new lanelet_ids
        at neighbors.

        Args:
          lanelet_pairs: List with tuples of lanelet_ids which should be concatenated.
          lanelet_pairs: list:

        Returns:
          : Dict with information which lanelet_id was converted to a new one.

        """

        new_lanelet_ids = dict()
        for pair in lanelet_pairs:
            if pair[0] == pair[1]:
                continue
            lanelet_1 = self.find_lanelet_by_id(pair[0])
            lanelet_2 = self.find_lanelet_by_id(pair[1])
            lanelet_1.concatenate(lanelet_2)

            self.remove_lanelet(pair[1])

            # each reference to lanelet_2 should point to lanelet_id
            # of new_lanelet instead
            self.update_lanelet_id_references(
                lanelet_2.lanelet_id, lanelet_1.lanelet_id
            )
            # update dict to show which lanelet_id changed to which
            new_lanelet_ids[pair[1]] = pair[0]

        return new_lanelet_ids

    def join_and_split_possible_lanes(self):
        """
        This method provides the functionality to modify the lane boundaries if
        a lane merges into another lane or splits from another lane.

        """

        # Condition for lane merge:
        # left and right vertices at end or beginning are the same
        join_and_split_lanelets = []
        for lanelet in self.lanelets:

            lanelet_split, lanelet_join = False, False
            if not lanelet.predecessor and np.allclose(
                lanelet.left_vertices[0], lanelet.right_vertices[0]
            ):
                lanelet_split = True

            if not lanelet.successor and np.allclose(
                lanelet.left_vertices[-1], lanelet.right_vertices[-1]
            ):
                lanelet_join = True

            if lanelet_join or lanelet_split:
                join_and_split_lanelets.append(
                    {
                        "lanelet": lanelet,
                        "is_split": lanelet_split,
                        "is_join": lanelet_join,
                    }
                )

        self._determine_js_adjacent_lanelet(join_and_split_lanelets)
        # list of ids of lanelets which are joined into/splitted from
        # used to avoid changing their boundaries
        global_adjacent_lanelets = [
            x.get("adjacent_lanelet").lanelet_id for x in join_and_split_lanelets
        ]
        for js_target in join_and_split_lanelets:
            lanelet = js_target.get("lanelet")
            if js_target.get("is_split"):
                self._perform_split_or_join(
                    True, js_target, global_adjacent_lanelets=global_adjacent_lanelets
                )
                if js_target.get("is_join"):
                    left_vertices = lanelet.left_vertices
                    right_vertices = lanelet.right_vertices
                    center_vertices = lanelet.center_vertices

            if js_target.get("is_join"):
                self._perform_split_or_join(
                    False, js_target, global_adjacent_lanelets=global_adjacent_lanelets
                )

            if js_target.get("is_join") and js_target.get("is_split"):
                half_length = int(left_vertices[:, 0].size / 2)
                lanelet.left_vertices = np.vstack(
                    (
                        left_vertices[:half_length, :],
                        lanelet.left_vertices[half_length:, :],
                    )
                )
                lanelet.right_vertices = np.vstack(
                    (
                        right_vertices[:half_length, :],
                        lanelet.right_vertices[half_length:, :],
                    )
                )
                lanelet.center_vertices = np.vstack(
                    (
                        center_vertices[:half_length, :],
                        lanelet.center_vertices[half_length:, :],
                    )
                )

    def _determine_js_adjacent_lanelet(self, join_and_split_lanelets: list):
        """Determine the neighbor which should be used for the join/split operation.

        I.e. the neighbor where the lanelet joins into and/or splits from.

        Args:
          join_and_split_lanelets: List of dicts which contain the lanelets
            and information about whether it is a join and/or a split.

        Note:
          Not yet implemented is the option that the lanelet can join into both
          or split from both neighbors. Now the left adjacent neighbor is preferred
          if possible.
        """

        # checked_lanelets is the check whether adjacent_lanelet
        # which will be used to join/split is a direct neighbor
        # of the lanelet or not
        for js_target in join_and_split_lanelets:
            lanelet = js_target.get("lanelet")
            adjacent_lanelets = Queue()
            checked_lanelets = 0
            if lanelet.adj_left is not None and lanelet.adj_left_same_direction:
                adjacent_lanelets.put(
                    {"lanelet_id": lanelet.adj_left, "linking_side": "right"}
                )
                checked_lanelets -= 1
            if lanelet.adj_right is not None and lanelet.adj_right_same_direction:
                adjacent_lanelets.put(
                    {"lanelet_id": lanelet.adj_right, "linking_side": "left"}
                )
                checked_lanelets -= 1

            while adjacent_lanelets.qsize() > 0:
                self._check_next_adj_lanelet(js_target, adjacent_lanelets)
                checked_lanelets += 1

                if checked_lanelets > 0:
                    js_target["single_lanelet_operation"] = True
                if js_target.get("linking_side"):
                    # found appropriate adjacent lanelet
                    break

        join_and_split_lanelets[:] = [
            x for x in join_and_split_lanelets if x.get("linking_side")
        ]

    def _check_next_adj_lanelet(self, js_target: dict, adjacent_lanelets: Queue):
        """Check if an adjacent lanelet is able to function as
        the lanelet which can be split from / joined into.

        If this is not the case (width is not greater 0), try to add next neighbor in same
        direction to adjacent_lanelets queue.

        Args:
          js_target: Dict containing information about the lanelet
            and whether it is a join or split.
          adj_target: Containing id of next potential adjacent lanelet
            and the side where it is linked to the splitting / joining
            lanelet.
        """
        adj_target = adjacent_lanelets.get()
        adj_lanelet = self.find_lanelet_by_id(adj_target.get("lanelet_id"))
        if not adj_lanelet:
            return
        linking_side = adj_target.get("linking_side")
        for option in ["is_split", "is_join"]:
            if js_target.get(option):
                adj_width = (
                    adj_lanelet.calc_width_at_start()
                    if option == "is_split"
                    else adj_lanelet.calc_width_at_end()
                )
                if adj_width > 0:
                    js_target["linking_side"] = linking_side
                    js_target[option[3::] + "_adj_width"] = adj_width
                    js_target["adjacent_lanelet"] = adj_lanelet
                else:
                    next_adj_neighbor = (
                        adj_lanelet.adj_left
                        if linking_side == "right"
                        else adj_lanelet.adj_right
                    )
                    if next_adj_neighbor:
                        adjacent_lanelets.put(
                            {
                                "lanelet_id": next_adj_neighbor,
                                "linking_side": linking_side,
                            }
                        )

    def _perform_split_or_join(
        self, is_split: bool, js_target: dict, global_adjacent_lanelets: list
    ):
        """Helper function to simplify split or join algorithm.

        Selects proper attributes depending on the scenario.

        Args:
          lanelet: Lanelet on which to perform algorithm.
          is_split: True if lanelet is splitting from other lanelet, else False
            if it is a join.
          global_adjacent_lanelets: Containing ids of all adjacent lanelets
            which are used to split from / join into.
        """
        lanelet = js_target.get("lanelet")

        adj_lanelet = js_target.get("adjacent_lanelet")
        split_and_join = js_target.get("is_split") and js_target.get("is_join")
        change_pos, change_width = lanelet.optimal_join_split_values(
            is_split=is_split, split_and_join=split_and_join
        )
        if is_split:
            self._perform_split_lanelets(
                js_target, change_pos, change_width, global_adjacent_lanelets
            )
            self.add_predecessors_to_lanelet(lanelet, adj_lanelet.predecessor)
        else:
            self._perform_join_lanelets(
                js_target, change_pos, change_width, global_adjacent_lanelets
            )
            self.add_successors_to_lanelet(lanelet, adj_lanelet.successor)

    def _perform_split_lanelets(
        self, js_target, change_pos, change_width, global_adjacent_lanelets
    ):
        adj_width = js_target.get("split_adj_width")
        lanelet = js_target.get("lanelet")
        optimal_split_pos = optimal_join_split_factor * adj_width
        if (
            change_pos == lanelet.length
            and optimal_split_pos > change_pos
            and not js_target.get("single_lanelet_operation")
        ):
            change_lanelets = [lanelet]
            change_length = [0, lanelet.length]
            adjacent_lanelets = [js_target.get("adjacent_lanelet")]

            while change_length[-1] < optimal_split_pos:
                lane = change_lanelets[-1]
                adjacent_lanelet = adjacent_lanelets[-1]
                if (
                    self.successor_is_neighbor_of_neighbors_successor(lane)
                    and lane.successor[0] not in global_adjacent_lanelets
                ):
                    successor = self.find_lanelet_by_id(lane.successor[0])
                    # break if width is smaller in next lanelet
                    if successor.calc_width_at_start() < lane.calc_width_at_end():
                        break
                    adj_successor = self.find_lanelet_by_id(
                        adjacent_lanelet.successor[0]
                    )
                    change_length.append(successor.length + change_length[-1])
                    change_lanelets.append(successor)
                    adjacent_lanelets.append(adj_successor)
                else:
                    break
            algo_list = []
            if optimal_split_pos > change_length[-1]:
                new_change_width = change_lanelets[-1].calc_width_at_end()
                for i, lane in enumerate(change_lanelets):
                    [dist_start, dist_end] = np.interp(
                        [0 + change_length[i], lane.length + change_length[i]],
                        [0, change_length[-1]],
                        [adj_width, new_change_width],
                    )
                    algo_list.append(
                        (
                            lane,
                            [0, lane.length],
                            [dist_start, dist_end],
                            adjacent_lanelets[i],
                        )
                    )

            else:
                end_pos = optimal_split_pos - change_length[-2]
                new_change_width = change_lanelets[-1].calc_width(end_pos)
                for i, lane in enumerate(change_lanelets[:-1:]):
                    [dist_start, dist_end] = np.interp(
                        [0 + change_length[i], lane.length + change_length[i]],
                        [0, optimal_split_pos],
                        [adj_width, new_change_width],
                    )
                    algo_list.append(
                        (
                            lane,
                            [0, lane.length],
                            [dist_start, dist_end],
                            adjacent_lanelets[i],
                        )
                    )

                # for last lanelet
                [dist_start] = np.interp(
                    [0 + change_length[-2]],
                    [0, optimal_split_pos],
                    [adj_width, new_change_width],
                )
                algo_list.append(
                    (
                        change_lanelets[-1],
                        [0, end_pos],
                        [dist_start, new_change_width],
                        adjacent_lanelets[-1],
                    )
                )
        else:
            algo_list = [
                (
                    lanelet,
                    [0, change_pos],
                    [adj_width, change_width],
                    js_target.get("adjacent_lanelet"),
                )
            ]

        self._move_borders_of_lanelets(algo_list, js_target.get("linking_side"))

    def _select_apt_join_lanelets(
        self, js_target, optimal_join_pos, global_adjacent_lanelets
    ):
        lanelet = js_target.get("lanelet")
        change_lanelets = [lanelet]
        change_length = [lanelet.length, 0]
        adjacent_lanelets = [js_target.get("adjacent_lanelet")]

        while change_length[-1] > optimal_join_pos:
            lane = change_lanelets[-1]
            adjacent_lanelet = adjacent_lanelets[-1]
            if (
                self.predecessor_is_neighbor_of_neighbors_predecessor(lane)
                and lane.predecessor[0] not in global_adjacent_lanelets
            ):
                predecessor = self.find_lanelet_by_id(lane.predecessor[0])
                adj_predecessor = self.find_lanelet_by_id(
                    adjacent_lanelet.predecessor[0]
                )
                change_length.append(change_length[-1] - predecessor.length)
                change_lanelets.append(predecessor)
                adjacent_lanelets.append(adj_predecessor)
            else:
                break

    def _perform_join_lanelets(
        self, js_target, change_pos, change_width, global_adjacent_lanelets
    ):
        lanelet = js_target.get("lanelet")
        adj_width = js_target.get("join_adj_width")
        optimal_join_pos = lanelet.length - optimal_join_split_factor * adj_width
        if (
            change_pos == 0
            and optimal_join_pos < change_pos
            and not js_target.get("single_lanelet_operation")
        ):
            change_lanelets = [lanelet]
            change_length = [lanelet.length, 0]
            adjacent_lanelets = [js_target.get("adjacent_lanelet")]

            while change_length[-1] > optimal_join_pos:
                lane = change_lanelets[-1]
                adjacent_lanelet = adjacent_lanelets[-1]
                if (
                    self.predecessor_is_neighbor_of_neighbors_predecessor(lane)
                    and lane.predecessor[0] not in global_adjacent_lanelets
                ):
                    predecessor = self.find_lanelet_by_id(lane.predecessor[0])
                    adj_predecessor = self.find_lanelet_by_id(
                        adjacent_lanelet.predecessor[0]
                    )
                    change_length.append(change_length[-1] - predecessor.length)
                    change_lanelets.append(predecessor)
                    adjacent_lanelets.append(adj_predecessor)
                else:
                    break
            algo_list = []

            if optimal_join_pos < change_length[-1]:
                new_change_width = change_lanelets[-1].calc_width_at_start()
                for i, lane in enumerate(change_lanelets):
                    [dist_start, dist_end] = np.interp(
                        [0 + change_length[i + 1], lane.length + change_length[i + 1]],
                        [change_length[-1], change_length[0]],
                        [new_change_width, adj_width],
                    )
                    algo_list.append(
                        (
                            lane,
                            [0, lane.length],
                            [dist_start, dist_end],
                            adjacent_lanelets[i],
                        )
                    )

            else:
                end_pos = optimal_join_pos - change_length[-1]
                new_change_width = change_lanelets[-1].calc_width(end_pos)
                for i, lane in enumerate(change_lanelets[:-1:]):
                    [dist_start, dist_end] = np.interp(
                        [0 + change_length[i + 1], lane.length + change_length[i + 1]],
                        [optimal_join_pos, change_length[0]],
                        [new_change_width, adj_width],
                    )
                    algo_list.append(
                        (
                            lane,
                            [0, lane.length],
                            [dist_start, dist_end],
                            adjacent_lanelets[i],
                        )
                    )
                # for last lanelet
                [dist_end] = np.interp(
                    [change_length[-2]],
                    [optimal_join_pos, change_length[0]],
                    [new_change_width, adj_width],
                )
                algo_list.append(
                    (
                        change_lanelets[-1],
                        [end_pos, change_lanelets[-1].length],
                        [new_change_width, dist_end],
                        adjacent_lanelets[-1],
                    )
                )

        else:
            algo_list = [
                (
                    lanelet,
                    [change_pos, lanelet.length],
                    [change_width, adj_width],
                    js_target.get("adjacent_lanelet"),
                )
            ]
        self._move_borders_of_lanelets(algo_list, js_target.get("linking_side"))

    def _move_borders_of_lanelets(self, algo_list: list, linking_side: str):
        """Move borders of lanelets in algo_list.

        Determine the adjacent lanelet before for each lanelet
        because it is needed for boundary control of the move_border
        function.

        Args:
          algo_list: List containing lanelets, distance and interval
            of how the border should be moved.
          linking_side: Indicates whether left or right border should
            be moved.
        """
        for lanelet, mirror_interval, distance, adjacent_lanelet in algo_list:
            lanelet.move_border(
                mirror_border=linking_side,
                mirror_interval=mirror_interval,
                distance=distance,
                adjacent_lanelet=adjacent_lanelet,
            )

    def predecessor_is_neighbor_of_neighbors_predecessor(
        self, lanelet: "ConversionLanelet"
    ) -> bool:
        """Checks if neighbors of predecessor are the successor of the adjacent neighbors
        of the lanelet.

        Args:
          lanelet: Lanelet to check neighbor requirement for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        if not self.has_unique_pred_succ_relation(-1, lanelet):
            return False

        predecessor = self.find_lanelet_by_id(lanelet.predecessor[0])
        return self.successor_is_neighbor_of_neighbors_successor(predecessor)

    def add_successors_to_lanelet(self, lanelet: ConversionLanelet, successor_ids: str):
        """Add a successor to a lanelet, but add the lanelet also to the predecessor
        of the succesor.

        Args:
          lanelet: Lanelet to add successor to.
          successor_ids: Id of successor to add to lanelet.
        """
        for successor_id in successor_ids:
            lanelet.successor.append(successor_id)
            successor = self.find_lanelet_by_id(successor_id)
            successor.predecessor.append(lanelet.lanelet_id)

    def add_predecessors_to_lanelet(
        self, lanelet: ConversionLanelet, predecessor_ids: str
    ):
        """Add a successor to a lanelet, but add the lanelet also to the predecessor
        of the succesor.

        Args:
          lanelet: Lanelet to add successor to.
          predecessor_id: Id of successor to add to lanelet.
        """
        for predecessor_id in predecessor_ids:
            lanelet.predecessor.append(predecessor_id)
            predecessor = self.find_lanelet_by_id(predecessor_id)
            predecessor.successor.append(lanelet.lanelet_id)

    def check_concatenation_potential(
        self, lanelet: ConversionLanelet, adjacent_direction: str
    ) -> list:
        """Check if lanelet could be concatenated with its successor.

        Args:
          lanelet: Lanelet to check concatenation potential with its successor
          adjacent_direction: "Left" or "Right", determinating which lanelet

        Returns:
          A list of pairs of lanelets which can be concatenated. None if it is not possible.

        """
        mergeable_lanelets = []
        neighbor_ok = self.successor_is_neighbor_of_neighbors_successor(lanelet)
        if not neighbor_ok:
            return None

        if adjacent_direction == "left":
            mergeable_lanelets.append((lanelet.lanelet_id, lanelet.successor[0]))
            if lanelet.adj_left is None:
                return mergeable_lanelets
            if lanelet.adj_left_same_direction:
                next_neighbor = self.find_lanelet_by_id(lanelet.adj_left)
                new_direction = "left"
            else:
                next_neighbor = self.find_lanelet_by_id(
                    self.find_lanelet_by_id(lanelet.adj_left).predecessor[0]
                )
                new_direction = "right"

        else:
            mergeable_lanelets.append((lanelet.lanelet_id, lanelet.successor[0]))
            if lanelet.adj_right is None:
                return mergeable_lanelets
            if lanelet.adj_right_same_direction:
                next_neighbor = self.find_lanelet_by_id(lanelet.adj_right)
                new_direction = "right"
            else:
                next_neighbor = self.find_lanelet_by_id(
                    self.find_lanelet_by_id(lanelet.adj_right).predecessor[0]
                )
                new_direction = "left"

        recursive_merge = self.check_concatenation_potential(
            next_neighbor, new_direction
        )
        if recursive_merge is None:
            return None

        mergeable_lanelets.extend(recursive_merge)
        return mergeable_lanelets

    def successor_is_neighbor_of_neighbors_successor(
        self, lanelet: ConversionLanelet
    ) -> bool:
        """Checks if neighbors of successor are the successor of the adjacent neighbors
        of the lanelet.

        Args:
          lanelet: Lanelet to check specified relation for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        if not self.has_unique_pred_succ_relation(1, lanelet):
            return False

        return self.adj_left_consistent_nb(lanelet) and self.adj_right_consistent_nb(
            lanelet
        )

    def has_unique_pred_succ_relation(
        self, direction: int, lanelet: ConversionLanelet
    ) -> bool:
        """Checks if lanelet has only one successor/predecessor and the
        successor/predecessor has only one predecessor/successor, s.t.
        it is a one-to-one relation.

        Args:
          direction: 1 if the successor should be checked.
        -1 (or all other values) for the predecessor.
          lanelet_network: Network to search for lanelets by ids.
          direction: int:
          lanelet_network: "LaneletNetwork":

        Returns:
          True if the relation is unique, False otherwise.

        """
        if direction == 1:
            neighbors = lanelet.successor
        else:
            neighbors = lanelet.predecessor

        # check if neighbor is only one
        if neighbors is None or len(neighbors) != 1:
            return False

        # get lanelet object with id
        neighbor = self.find_lanelet_by_id(neighbors[0])

        # get the neighbor of the neighbor
        nb_neighbor = neighbor.predecessor if direction == 1 else neighbor.successor

        # check if nb_neighbor has one neighbor in proper direction
        if nb_neighbor is None or len(nb_neighbor) != 1:
            return False

        # return True if these ids are the same (they should be!)
        return lanelet.lanelet_id == nb_neighbor[0]

    def adj_right_consistent_nb(self, lanelet: ConversionLanelet) -> bool:
        """Checks if right neighbor of successor is the successor
        of the right adjacent neighbor of the lanelet.

        Args:
          lanelet: Lanelet to check specified relation for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        successor = self.find_lanelet_by_id(lanelet.successor[0])
        adj_right = self.find_lanelet_by_id(lanelet.adj_right)
        if adj_right:
            if lanelet.adj_right_same_direction:
                if not self.has_unique_pred_succ_relation(1, adj_right):
                    return False
                if adj_right.successor[0] != successor.adj_right:
                    return False
            else:
                if not lanelet.has_unique_pred_succ_relation(-1, adj_right):
                    return False
                if adj_right.predecessor[0] != successor.adj_right:
                    return False
        else:
            return successor.adj_right is None
        return True

    def adj_left_consistent_nb(self, lanelet: ConversionLanelet) -> bool:
        """Checks if left neighbor of successor is the successor of the
        left adjacent neighbor of the lanelet.

        Args:
          lanelet: Lanelet to check specified relation for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        successor = self.find_lanelet_by_id(lanelet.successor[0])
        adj_left = self.find_lanelet_by_id(lanelet.adj_left)
        if adj_left:
            if lanelet.adj_left_same_direction:
                if not self.has_unique_pred_succ_relation(1, adj_left):
                    return False
                if adj_left.successor[0] != successor.adj_left:
                    return False
            else:
                if not self.has_unique_pred_succ_relation(-1, adj_left):
                    return False
                if adj_left.predecessor[0] != successor.adj_left:
                    return False
        else:
            return successor.adj_left is None
        return True


class _JoinSplitTarget:
    """Class to integrate transforming of lanelet borders due to joining / splitting."""

    def __init__(
        self,
        lanelet_network: ConversionLaneletNetwork,
        lanelet: ConversionLanelet,
        split: bool,
        join: bool,
    ):
        self.lanelet = lanelet
        self.lanelet_network = lanelet_network
        self.split = split
        self.join = join
        self.linking_side = None
        self.adjacent_lanelet = None

    def split_and_join(self) -> bool:
        """Lanelet splits at start and joins at end.

        Returns:
          True if it has a join and a split.
        """
        return self.split and self.join

    def move_lanelet_borders(self):
        """Move borders of lanelets for appropriate joins and splits."""
        # TODO: implement this and use class to simplify code in ConversionLaneletNetwork
