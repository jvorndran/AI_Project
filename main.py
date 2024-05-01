from WareHouseSim import WarehouseSimulation


def main():

    test_enviorments = [
        {
            'grid_size': 50,
            'num_robots': 5,
            'num_obstacles': 0,
            'cell_size': 10
        },
        {
            'grid_size': 30,
            'num_robots': 8,
            'num_obstacles': 50,
            'cell_size': 20
        },
        {
            'grid_size': 30,
            'num_robots': 15,
            'num_obstacles': 20,
            'cell_size': 20
        },
        {
            'grid_size': 30,
            'num_robots': 5,
            'num_obstacles': 40,
            'cell_size': 20
        }
    ]

    for test in test_enviorments:
        print(f'Starting simulation with grid size {test['grid_size']}, {test['num_robots']} robots, and {test['num_obstacles']} obstacles.')
        sim = WarehouseSimulation(test['grid_size'], test['cell_size'], test['num_robots'], test['num_obstacles'])
        sim.run()


if __name__ == '__main__':
    main()

