from WareHouseSim import WarehouseSimulation


def main():
    grid_size = 40
    num_robots = 10
    num_obstacles = 40
    cell_size = 20

    print(f'Starting simulation with grid size {grid_size}, {num_robots} robots, and {num_obstacles} obstacles.')
    sim = WarehouseSimulation(grid_size, cell_size, num_robots, num_obstacles)
    sim.run()


if __name__ == '__main__':
    main()
