"""Run only the cabinet task FK/state estimator (no policy or controller)."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fr3_husky_nn_policy",
            executable="cabinet_book_insert_fk_node",
            name="cabinet_book_insert_fk_node",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("fr3_husky_nn_policy"),
                "config",
                "dual_fr3_cabinet_book_insert_fk.yaml",
            ])],
        ),
    ])
