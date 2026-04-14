import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Cesty k tvým souborům
    pkg_student = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(pkg_student, 'rviz', 'config.rviz')

    # 2. Cesta k modelu robota (URDF) uvnitř simulátoru
    # (Tady předpokládáme, že je to ve složce simulátoru. Název 'tiago.urdf' případně uprav
    # podle toho, jak se ten soubor reálně jmenuje ve složce mpc_rbt_simulator/urdf/)
    pkg_simulator = get_package_share_directory('mpc_rbt_simulator')
    urdf_file = os.path.join(pkg_simulator, 'resources', 'tiago_model_ros.urdf')

    warehouse_manager = Node(
        package='mpc_rbt_student',
        executable='warehouse_manager',
        name='warehouse_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    bt_server = Node(
        package='mpc_rbt_student',
        executable='bt_server',
        name='bt_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            os.path.join(pkg_student, 'config', 'bt_server.yaml')
        ]
    )   

    # Načtení obsahu URDF souboru do proměnné
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # --- UZEL 1: Tvoje dnešní matematika (Odometrie a TF) ---
        Node(
            package='mpc_rbt_student',
            executable='localization_node',
            name='localization_node',
            output='screen'
        ),

        # --- UZEL 2: Poskládání robota v prostoru (TF pro LiDAR a kola) ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # --- UZEL 3: RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # --- UZEL 3: A* ---
        Node(
            package='mpc_rbt_student',
            executable='planning_node',
            name='planning_node',
            output='screen'
        ),

        # --- UZEL 4: motion control ---
        Node(
            package='mpc_rbt_student',
            executable='motion_control_node',
            name='motion_control_node',
            output='screen'
        ),
        warehouse_manager,
        bt_server
    ])